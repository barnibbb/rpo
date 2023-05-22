#include "dose_calculator.h"

using namespace std;

namespace rpo
{
    using NodePtr = rpo::AugmentedOcTreeNode*;

    DoseCalculator::DoseCalculator(const AugmentedOcTree& augmented_model)
    {
        m_augmented_model = make_shared<AugmentedOcTree>(augmented_model);

        m_parameters = m_augmented_model->getParameters();
    }


    void DoseCalculator::computeDoseForPlan(RadiationPlan& radiation_plan)
    {
        const int element_size = m_parameters.plan_element_size;

        vector<double> elements = radiation_plan.first;

        vector<ExposureMap> exposure_maps(m_parameters.number_of_positions);

        const double z = m_parameters.ground_level + m_parameters.resolution / 2 +
            m_parameters.lamp_offset + m_parameters.lamp_height / 2;

        for (int i = 0; i < elements.size(); i += element_size)
        {
            point3d lamp_position(elements[i], elements[i + 1], m_parameters.ground_level);

            OcTreeKey key; 

            m_augmented_model->coordToKeyChecked(lamp_position, m_parameters.depth, key);

            if (m_augmented_model->isGroundZoneElement(key))
            {
                lamp_position.z() = z;

                PlanElement plan_element { lamp_position, elements[i + 2] };

                exposure_maps[i / element_size] = computeDoseForPlanElement(plan_element);
            }
        }

        int over_limit = 0;

        for (auto& element : exposure_maps[0])
        {
            for (int i = 1; i < exposure_maps.size(); ++i)
            {
                element.second += exposure_maps[i][element.first];
            }

            if (element.second > m_parameters.exposure_limit)
            {
                over_limit += 1;
            }
        }

        radiation_plan.second = static_cast<double>(over_limit) / 
            static_cast<double>(m_augmented_model->getReachableElements().size());
    }


    ExposureMap DoseCalculator::computeDoseForPlanElement(const PlanElement& plan_element)
    {
        ExposureMap dose_map;

        OcTreeKey plan_element_key;

        m_augmented_model->coordToKeyChecked(plan_element.first, m_parameters.depth, plan_element_key);

        if (m_parameters.precompute_irradiance)
        {
            double min_distance = numeric_limits<double>::max();

            OcTreeKey grid_key;

            for (const auto& element : m_radiation_positions)
            {
                const point3d grid_position = m_augmented_model->keyToCoord(element, m_parameters.depth);
            
                const double distance = (grid_position - plan_element.first).norm();

                if (distance < min_distance)
                {
                    min_distance = distance;
                    grid_key = element;
                }
            }

            dose_map = loadIrradianceMap(grid_key);
        }
        else
        {
            if (m_radiation_positions.find(plan_element_key) != m_radiation_positions.end())
            {
                dose_map = loadIrradianceMap(plan_element_key);
            }
            else
            {
                dose_map = computeIrradianceForPosition(plan_element.first);

                saveIrradianceMap(plan_element_key, dose_map);
            }
        }

        for (auto& element : dose_map)
        {
            element.second *= plan_element.second;
        }

        return dose_map;
    }


    ExposureMap DoseCalculator::computeIrradianceForPosition(const point3d& lamp_position)
    {
        ExposureMap irradiance_map;
    
        KeySet reachable_elements = m_augmented_model->getReachableElements();

        for (const auto& element : reachable_elements)
        {
            irradiance_map[element] = computeIrradianceForElement(lamp_position, element);
        }

        return irradiance_map;
    }


    double DoseCalculator::computeIrradianceForElement(const point3d& lamp_position, const OcTreeKey& key) const
    {
        const point3d point = m_augmented_model->keyToCoord(key, m_parameters.depth);

        const double distance = (point - lamp_position).norm();

        if ((!isnan(distance) && distance < m_parameters.lamp_range))
        {
            NodePtr node = m_augmented_model->search(key, m_parameters.depth);

            const point3d normal = node->getNormal();

            if (!isnan(normal.norm()))
            {
                double irradiance = 0;

                const double L = m_parameters.resolution;

                const double coefficient = m_parameters.lamp_power / 
                    (4 * M_PI * m_parameters.lamp_height);

                point3d center = lamp_position;

                for (int i = 1; i < m_ray_origins.size() - 1; ++i)
                {
                    center.z() = m_ray_origins[i];

                    if (!m_parameters.expand_model || (m_parameters.expand_model && 
                        (point - center).dot(normal) <= 0))
                    {
                        const point3d pd = point - center;

                        const double F_lower = (normal.z() * (pow(pd.x(), 2) + pow(pd.y(), 2)) +
                            (-L/2 - pd.z()) * (normal.x() * pd.x() + normal.y() * pd.y())) /
                            ((pow(pd.x(), 2) + pow(pd.y(), 2)) * 
                            pow(pow(-L/2 - pd.z(), 2) + pow(pd.x(), 2) + pow(pd.y(), 2), 0.5));

                        const double F_upper = (normal.z() * (pow(pd.x(), 2) + pow(pd.y(), 2)) +
                            (L/2 - pd.z()) * (normal.x() * pd.x() + normal.y() * pd.y())) /
                            ((pow(pd.x(), 2) + pow(pd.y(), 2)) * 
                            pow(pow(L/2 - pd.z(), 2) + pow(pd.x(), 2) + pow(pd.y(), 2), 0.5));

                        bool visible = false;

                        switch (m_parameters.ray_traing)
                        {
                            case RayTracing::None:
                            {
                                visible = true;
                                break;
                            }
                            case RayTracing::TwoDimensional:
                            {
                                visible = compute2DVisibility();
                                break;
                            }
                            case RayTracing::ThreeDimensional:
                            {
                                visible = compute3DVisibility(center, point, distance);
                                break;
                            }
                            default:
                            {
                                visible = false;
                                break;
                            }
                        }

                        irradiance += visible ? (coefficient * abs(F_upper - F_lower)) : 0;
                    }
                }

                return irradiance;
            }
        }

        return 0;
    }


    bool DoseCalculator::compute2DVisibility() const
    {
        // TODO
        return true;
    }


    bool DoseCalculator::compute3DVisibility(const point3d& lamp_position, const point3d& element, const double distance) const
    {
        const double resolution = m_parameters.resolution + 0.01;

        const int depth = m_parameters.depth;

        const point3d direction = lamp_position - element;

        double offset = (direction.x() >= 0) ? resolution : -resolution;

        point3d origin = element + point3d(offset, 0, 0);

        point3d target_direction = lamp_position - origin;

        point3d end_point;

        if (m_augmented_model->checkRayCast(false, lamp_position, origin, 
            target_direction, end_point, distance, depth, resolution, true))
        {
            offset = (direction.y() >= 0) ? resolution : -resolution;

            origin = element + point3d(0, offset, 0);

            target_direction = lamp_position - origin;

            if (m_augmented_model->checkRayCast(false, lamp_position, origin, 
                target_direction, end_point, distance, depth, resolution, true))
            {
                offset = (direction.z() >= 0) ? resolution : -resolution;

                origin = element + point3d(0, 0, offset);
                
                target_direction = lamp_position - origin; 
                
                if (m_augmented_model->checkRayCast(true, lamp_position, origin, 
                    target_direction, end_point, distance, depth, resolution, true))
                {
                    return true;
                }
            }
            else
            {
                return true;
            }
        } 
        else
        {
            return true;
        }          

        return false;
    }


    ExposureMap DoseCalculator::loadIrradianceMap(const OcTreeKey& plan_element_key) const
    {
        ExposureMap irradiance_map;

        const string input_file = m_parameters.precomputation_folder +
            to_string(plan_element_key[0]) + "_" + 
            to_string(plan_element_key[1]) + "_" +
            to_string(plan_element_key[2]) + ".txt";

        ifstream file(input_file);

        if (file.is_open())
        {
            string line;

            while (getline(file, line))
            {
                vector<string> data;

                boost::split(data, line, boost::is_any_of(" "));

                OcTreeKey key(
                    static_cast<uint16_t>(stoi(data[0])),
                    static_cast<uint16_t>(stoi(data[1])),
                    static_cast<uint16_t>(stoi(data[2]))
                );

                irradiance_map[key] = stod(data[3]);
            }

            file.close();
        }

        return irradiance_map;
    }
    
    
    void DoseCalculator::saveIrradianceMap(const OcTreeKey& plan_element_key, const ExposureMap& irradiance_map)
    {
        const string output_file = m_parameters.precomputation_folder + 
            to_string(plan_element_key[0]) + "_" + 
            to_string(plan_element_key[1]) + "_" + 
            to_string(plan_element_key[2]) + ".txt";
    
        ofstream file(output_file);

        if (file.is_open())
        {
            for (const auto& element : irradiance_map)
            {
                file << element.first[0] << ' ' << element.first[1] << ' ' << element.first[2]
                     << ' ' << element.second << '\n';
            }

            m_radiation_positions.insert(plan_element_key);

            file.close();
        }
    }


    void DoseCalculator::preComputeIrradianceMaps()
    {
        const double z = m_parameters.ground_level + m_parameters.resolution / 2 +
            m_parameters.lamp_offset + m_parameters.lamp_height / 2;

        vector<OcTreeKey> grid_vector;

        //for (const auto& key : m_augmented_model->m_grid_elements)
        for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(),
            end = m_augmented_model->end_leafs(); it != end; ++it)
        {
            if (NodePtr node = m_augmented_model->search(it.getKey()); node != nullptr)
            {
                if (node->isGrid())
                {
                    grid_vector.push_back(it.getKey());
                }
            }
        }

        #pragma omp parallel for
        for (int i = 0; i < grid_vector.size(); ++i)
        {
            point3d position = m_augmented_model->keyToCoord(grid_vector[i], m_parameters.depth);

            position.z() = z;

            ExposureMap irradiance_map = computeIrradianceForPosition(position);

            OcTreeKey plan_element_key;

            m_augmented_model->coordToKeyChecked(position, m_parameters.depth, plan_element_key);

            saveIrradianceMap(plan_element_key, irradiance_map);
        }
    }


    void DoseCalculator::deleteIrradianceMaps() const
    {
        for (const auto& position_key : m_radiation_positions)
        {
            const string file = m_parameters.precomputation_folder + 
                to_string(position_key[0]) + "_" + 
                to_string(position_key[1]) + "_" + 
                to_string(position_key[2]) + ".txt";

            const string command = "rm " + file;

            system(command.c_str());
        }
    }


    void DoseCalculator::computeRayOrigins()
    {
        double height = m_parameters.ground_level + 
            m_parameters.resolution / 2.0 + m_parameters.lamp_offset;

        const double top_of_lamp = m_parameters.ground_level + 
            m_parameters.resolution / 2.0 + m_parameters.lamp_offset + 
            m_parameters.lamp_height;

        point3d continuous_height(0, 0, height);

        if (OcTreeKey key; m_augmented_model->coordToKeyChecked(continuous_height, m_parameters.depth, key))
        {
            point3d discrete_height = m_augmented_model->keyToCoord(key, m_parameters.depth);

            double height = discrete_height.z();

            while (height < top_of_lamp)
            {
                m_ray_origins.push_back(height);

                cout << height << endl;

                height += m_parameters.resolution;
            }
        }
    }
}
