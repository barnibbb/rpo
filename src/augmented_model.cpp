#include "augmented_model.h"

using namespace std;

namespace rpo
{
    AugmentedModel::AugmentedModel(const Parameters& parameters)
    {
        m_parameters = make_shared<Parameters>(parameters);

        ifstream file(m_parameters->model_path);

        if (file.is_open())
        {
            m_color_octree.reset(dynamic_cast<ColorOcTree*>(AbstractOcTree::read(file)));

            for (ColorOcTree::leaf_iterator it = m_color_octree->begin_leafs(), 
                end = m_color_octree->end_leafs(); it != end; ++it)
            {
                // All nodes that are pruned (i.e. are on a smaller depth level) are expanded.
                if (it.getDepth() < m_parameters->depth)
                {
                    NodePtr node = m_color_octree->search(it.getKey(), it.getDepth());

                    if (node != nullptr)
                    {
                        m_color_octree->expandNode(node);
                    }
                }
            }

            // Storing the metric limits of the 3D model.
            m_color_octree->getMetricMin(
                m_model_size.min_x, m_model_size.min_y, m_model_size.min_z);
            m_color_octree->getMetricMax(
                m_model_size.max_x, m_model_size.max_y, m_model_size.max_z);

            if (m_parameters->delete_ceiling)
            {
                deleteCeiling();
            }

            if (m_parameters->smooth_model)
            {
                smoothModel();
            }

            if (m_parameters->expand_model)
            {
                expandModel();
            }

            // Storing the metric limits of the 3D model.
            m_color_octree->getMetricMin(
                m_model_size.min_x, m_model_size.min_y, m_model_size.min_z);
            m_color_octree->getMetricMax(
                m_model_size.max_x, m_model_size.max_y, m_model_size.max_z);

            file.close();
        }
        else
        {
            throw runtime_error { "Could not load model file!" };
        }
    }


    void AugmentedModel::deleteCeiling()
    {   
        KeySet elements_to_delete;

        for (ColorOcTree::leaf_iterator it = m_color_octree->begin_leafs(), 
            end = m_color_octree->end_leafs(); it != end; ++it)
        {     
            if (it.getCoordinate().z() >= (m_model_size.max_z - 0.4))
            {
                elements_to_delete.insert(it.getKey());
            }
        }

        for (const auto& key : elements_to_delete)
        {
            m_color_octree->deleteNode(key, m_parameters->depth);
        }
    }


    void AugmentedModel::smoothModel()
    {
        const float resolution = m_parameters->resolution;

        const vector<point3d> steps { 
            { -resolution, 0, 0 }, { resolution, 0, 0 }, 
            { 0, -resolution, 0 }, { 0, resolution, 0 }
        };

        KeySet elements_to_delete, checked_elements;

        for (ColorOcTree::leaf_iterator it = m_color_octree->begin_leafs(), end = m_color_octree->end_leafs(); it != end; ++it)
        {
            KeySet elements_of_plane, unchecked_elements, elements_to_check;

            elements_of_plane.insert(it.getKey());

            unchecked_elements.insert(it.getKey());

            while (unchecked_elements.size() > 0)
            {
                for (const auto& element : unchecked_elements)
                {
                    checked_elements.insert(element);

                    point3d center = m_color_octree->keyToCoord(element);

                    for (const auto& step : steps)
                    {
                        point3d neighbor = center + step;

                        OcTreeKey neighbor_key;

                        m_color_octree->coordToKeyChecked(neighbor, m_parameters->depth, neighbor_key);

                        if (NodePtr node = m_color_octree->search(neighbor_key, m_parameters->depth); node != nullptr)
                        {
                            if (elements_of_plane.find(neighbor_key) == elements_of_plane.end())
                            {
                                elements_of_plane.insert(neighbor_key);
                            }

                            if (checked_elements.find(neighbor_key)   == checked_elements.end()   &&
                                elements_to_check.find(neighbor_key)  == elements_to_check.end())
                            {
                                elements_to_check.insert(neighbor_key);
                            }
                        }
                    }
                }

                if (unchecked_elements.size() > 0)
                {
                    unchecked_elements.erase(unchecked_elements.begin(), unchecked_elements.end());
                }

                for (const auto& key : elements_to_check)
                {
                    unchecked_elements.insert(key);
                }    

                if (elements_to_check.size() > 0)
                {
                    elements_to_check.erase(elements_to_check.begin(), elements_to_check.end());
                }
            }

            if (elements_of_plane.size() > 5)
            {
                for (const auto& element : elements_of_plane)
                {
                    point3d lower = m_color_octree->keyToCoord(element);

                    point3d middle = lower + point3d(0, 0, resolution);

                    point3d upper = middle + point3d(0, 0, resolution);

                    if (NodePtr middle_node = m_color_octree->search(middle, m_parameters->depth); middle_node != nullptr)
                    {
                        if (NodePtr upper_node = m_color_octree->search(upper, m_parameters->depth); upper_node == nullptr)
                        {
                            bool ok = true;

                            for (const auto& step : steps)
                            {
                                point3d neighbor = middle + step;

                                if (NodePtr n_node = m_color_octree->search(neighbor, m_parameters->depth); n_node == nullptr)
                                {
                                    ok = false;
                                    break;
                                }
                            }

                            if (!ok)
                            {
                                OcTreeKey middle_key;

                                m_color_octree->coordToKeyChecked(middle, m_parameters->depth, middle_key);

                                if (elements_to_delete.find(middle_key) == elements_to_delete.end())
                                {
                                    elements_to_delete.insert(middle_key);
                                }
                            }
                        }
                    }
                }
            }            
        }

        for (const auto& element : elements_to_delete)
        {
            NodePtr node = m_color_octree->search(element, m_parameters->depth);
            
            if (node != nullptr)
            {
                m_color_octree->deleteNode(element, m_parameters->depth);
            }
        }
    }


    void AugmentedModel::expandModel()
    {
        m_parameters->depth += 1;
        m_parameters->resolution /= 2;

        m_color_octree->expand();

        for (ColorOcTree::leaf_iterator it = m_color_octree->begin_leafs(), 
            end = m_color_octree->end_leafs(); it != end; ++it)
        {
            NodePtr node = m_color_octree->search(it.getKey(), m_parameters->depth - 1);

            if (node != nullptr)
            {
                m_color_octree->expandNode(node);
            }
        }
    }


    void AugmentedModel::computeGroundLevel()
    {
        map<double, int> height_map;

        // First it is counted how many voxels belong to each height.
        for (ColorOcTree::leaf_iterator it = m_color_octree->begin_leafs(), 
            end = m_color_octree->end_leafs(); it != end; ++it)
        {
            const point3d point = it.getCoordinate();

            if (NodePtr node = m_color_octree->search(point, m_parameters->depth); node != nullptr)
            {
                node->setColor(0, 0, 0);
            }

            if (height_map.find(point.z()) == height_map.end())
            {
                height_map[point.z()] = 1;
            }
            else
            {
                height_map[point.z()] += 1;
            }
        }

        int level_counter = 0;

        // The ground level will be the first or second (for fine resolution)
        // layer which contains at least 10% of all model elements.
        for (const auto& height : height_map)
        {
            if (height.second > 0.05 * m_color_octree->getNumLeafNodes())
            {
                ++level_counter;
            }

            if ((!m_parameters->expand_model && level_counter == 1) || 
                 (m_parameters->expand_model && level_counter == 2))
            {
                m_parameters->ground_level = height.first;

                break;
            }
        }
    }


    void AugmentedModel::computeGroundZone()
    {
        const float resolution = m_parameters->resolution;

        KeySet ground_zone;

        for (ColorOcTree::leaf_iterator it = m_color_octree->begin_leafs(), 
            end = m_color_octree->end_leafs(); it != end; ++it)
        {
            point3d point = it.getCoordinate();

            if (point.z() == m_parameters->ground_level)
            {
                m_ground_level_elements.insert(it.getKey());

                point.z() += resolution;

                bool ground = true;

                const double lamp_top = m_parameters->ground_level + m_parameters->resolution +
                    m_parameters->lamp_offset + m_parameters->lamp_height;

                while (point.z() < lamp_top)
                {
                    NodePtr node = m_color_octree->search(point);

                    if (node == nullptr)
                    {
                        point.z() += resolution;
                    }
                    else
                    {
                        ground = false;
                        break;
                    }
                }

                if (ground)
                {
                    ground_zone.insert(it.getKey());
                }
            }
        }

        for (const auto& key : ground_zone)
        {
            const point3d center = m_color_octree->keyToCoord(key, m_parameters->depth);

            bool ground = true;

            // For each possible ground zone element it checked whether its
            // neighbors are also ground zone elements. If all neighbors in
            // a given region also belong to the ground zone, then the center
            // is considered as real ground zone element.
            int min_dist = static_cast<int>(
                m_parameters->min_distance_from_obstacles / m_parameters->resolution);

            for (int i = -min_dist; i <= min_dist; ++i)
            {
                for (int j = -min_dist; j <= min_dist; ++j)
                {
                    const point3d neighbor = center + point3d(i * resolution, j * resolution, 0);

                    if (OcTreeKey neighbor_key; m_color_octree->coordToKeyChecked(
                        neighbor, m_parameters->depth, neighbor_key))
                    {
                        if (ground_zone.find(neighbor_key) == ground_zone.end())
                        {
                            ground = false;
                            break;
                        }
                    }
                }
            }

            if (ground)
            {
                if (NodePtr node = m_color_octree->search(key); node != nullptr)
                {
                    m_ground_zone_elements.insert(key);
                }
            }
        }
    }


    void AugmentedModel::computeReachableElements()
    {
        const float resolution = m_parameters->resolution;

        const vector<point3d> steps {
            { -resolution, 0, 0 }, { resolution, 0, 0 }, 
            { 0, -resolution, 0 }, { 0, resolution, 0 }, 
            { 0, 0, -resolution }, { 0, 0, resolution } };

        const vector<int> inverse_index { 1, 0, 3, 2, 5, 4 };

        KeySet unchecked_points, checked_points;

        point3d start_point(
            m_parameters->reachable_region_x,
            m_parameters->reachable_region_y, 
            m_parameters->ground_level + m_parameters->resolution / 2.0 + 
            m_parameters->lamp_offset + m_parameters->lamp_height / 2.0);

        OcTreeKey start_key;

        m_color_octree->coordToKeyChecked(start_point, m_parameters->depth, start_key);

        unchecked_points.insert(start_key);

        checked_points.insert(start_key);

        while (unchecked_points.size() > 0)
        {
            KeySet points_to_add;

            for (const auto& center_key : unchecked_points)
            {
                point3d center_point =  m_color_octree->keyToCoord(center_key, m_parameters->depth);

                for (int s = 0; s < steps.size(); ++s)
                {
                    if (!(center_point.z() < m_parameters->lamp_offset && s == 5))
                    {
                        point3d neighbor_point = center_point + steps[s];

                        if (neighbor_point.x() >= m_model_size.min_x && neighbor_point.x() <= m_model_size.max_x &&
                            neighbor_point.y() >= m_model_size.min_y && neighbor_point.y() <= m_model_size.max_y &&
                            neighbor_point.z() >= m_model_size.min_z && neighbor_point.z() <= m_model_size.max_z)
                        {
                            OcTreeKey neighbor_key;

                            m_color_octree->coordToKeyChecked(neighbor_point, m_parameters->depth, neighbor_key);

                            if (checked_points.find(neighbor_key) == checked_points.end())
                            {
                                if (NodePtr node = m_color_octree->search(neighbor_key, m_parameters->depth); node == nullptr)
                                {
                                    if (points_to_add.find(neighbor_key) == points_to_add.end()) 
                                    {
                                        points_to_add.insert(neighbor_key);

                                        checked_points.insert(neighbor_key);
                                    }
                                }
                                else
                                {
                                    if (m_reachable_elements.find(neighbor_key) == m_reachable_elements.end() &&
                                        isInsideBoundaries(neighbor_point, inverse_index[s]))
                                    {
                                        m_reachable_elements.insert(neighbor_key);

                                        checked_points.insert(neighbor_key);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if (unchecked_points.size() > 0)
            {
                unchecked_points.erase(unchecked_points.begin(), unchecked_points.end());
            }

            for (const auto& point : points_to_add)
            {
                unchecked_points.insert(point);
            }

            if (points_to_add.size() > 0)
            {
                points_to_add.erase(points_to_add.begin(), points_to_add.end());
            }
        }
    }


    void AugmentedModel::computeNormals()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr octree_points (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr octree_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>());

        for (ColorOcTree::leaf_iterator it = m_color_octree->begin_leafs(),
            end = m_color_octree->end_leafs(); it != end; ++it)
        {
            const point3d point = it.getCoordinate();

            octree_points->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
        }

        kdtree->setInputCloud(octree_points);

        normal_estimation.setInputCloud(octree_points);
        normal_estimation.setSearchMethod(kdtree);
        normal_estimation.setRadiusSearch(2 * m_parameters->resolution);
        normal_estimation.compute(*octree_normals);

        for (size_t i = 0; i < octree_points->size(); ++i)
        {
            const pcl::PointXYZ pcl_point = octree_points->points[i];

            const point3d point(pcl_point.data[0], pcl_point.data[1], pcl_point.data[2]);

            if (OcTreeKey key; m_color_octree->coordToKeyChecked(point, m_parameters->depth, key))
            {
                const pcl::Normal normal = octree_normals->at(i);

                m_surface_normals[key] = point3d(normal.normal_x, normal.normal_y, normal.normal_z);
            }
        }
    }


    void AugmentedModel::computePlanes()
    {
        const float resolution = m_parameters->resolution;

        const int depth = m_parameters->depth;

        // Modifies surface normals on horizontal surfaces
        if (m_parameters->modify_normals)
        {
            for (const auto& element : m_reachable_elements)
            {
                point3d point = m_color_octree->keyToCoord(element, depth);

                if (NodePtr node = m_color_octree->search(
                    point + point3d(0, 0, resolution), depth); node == nullptr)
                {
                    bool plane_element = true;

                    for (int x = -1; x < 2; ++x)
                    {
                        for (int y = -1; y < 2; ++y)
                        {
                            if (NodePtr neighbor_node = m_color_octree->search(
                                point + point3d(x * resolution,y * resolution, 0), depth);
                                neighbor_node == nullptr)
                            {
                                plane_element = false;
                            }
                        }
                    }

                    if (plane_element)
                    {
                        m_surface_normals[element] = point3d(0, 0, 1);
                    }
                }
            }

            for (const auto& element : m_reachable_elements)
            {
                point3d point = m_color_octree->keyToCoord(element, depth);

                if (NodePtr node = m_color_octree->search(element, depth); node != nullptr)
                {
                    point3d normal = m_surface_normals[element];

                    if (normal.x() > 0)
                    {
                        if (NodePtr neighbor_node = m_color_octree->search(
                            point + point3d(resolution, 0, 0), depth); neighbor_node != nullptr)
                        {
                            m_surface_normals[element] = point3d(-normal.x(), normal.y(), normal.z());
                        }
                    }
                    else if (normal.x() < 0)
                    {
                        if (NodePtr neighbor_node = m_color_octree->search(
                            point + point3d(-resolution, 0, 0), depth); neighbor_node != nullptr)
                        {
                            m_surface_normals[element] = point3d(-normal.x(), normal.y(), normal.z());
                        }
                    }

                    normal = m_surface_normals[element];

                    if (normal.y() > 0)
                    {
                        if (NodePtr neighbor_node = m_color_octree->search(
                            point + point3d(0, resolution, 0), depth); neighbor_node != nullptr)
                        {
                            m_surface_normals[element] = point3d(normal.x(), -normal.y(), normal.z());
                        }
                    }
                    else if (normal.y() < 0)
                    {
                        if (NodePtr neighbor_node = m_color_octree->search(
                            point + point3d(0, -resolution, 0), depth); neighbor_node != nullptr)
                        {
                            m_surface_normals[element] = point3d(normal.x(), -normal.y(), normal.z());
                        }
                    }

                    normal = m_surface_normals[element];

                    if (normal.z() > 0)
                    {
                        if (NodePtr neighbor_node = m_color_octree->search(
                            point + point3d(0, 0, resolution), depth); neighbor_node != nullptr)
                        {
                            m_surface_normals[element] = point3d(normal.x(), normal.y(), -normal.z());
                        }
                    }
                    else if (normal.z() < 0)
                    {
                        if (NodePtr neighbor_node = m_color_octree->search(
                            point + point3d(0, 0, -resolution), depth); neighbor_node != nullptr)
                        {
                            m_surface_normals[element] = point3d(normal.x(), normal.y(), -normal.z());
                        }
                    }
                }
            }
        }


        // Based on the angle to the vertical direction, segments horizontal and vertical surfaces
        for (const auto& element : m_reachable_elements)
        {
            static const point3d vertical(0, 0, 1);

            const double quarter = M_PI / 2.0;

            const double angle_to_vertical = abs(m_surface_normals[element].angleTo(vertical));
                
            point3d point = m_color_octree->keyToCoord(element, depth);

            if ((angle_to_vertical < M_PI / 18.0 || angle_to_vertical > (M_PI * 17.0 / 18.0)) && point.z() > m_parameters->ground_level)
            {
                m_horizontal_elements.insert(element);
            }
            else if (angle_to_vertical > (quarter - M_PI / 18.0) && angle_to_vertical < (quarter + M_PI / 18.0))
            {
                m_vertical_elements.insert(element);
            }
            else
            {
                m_general_elements.insert(element);
            }                
        }
    }


    void AugmentedModel::computeGridElements()
    {
        KeySet grid_elements;

        for (ColorOcTree::tree_iterator it = m_color_octree->begin_tree(), 
            end = m_color_octree->end_tree(); it != end; ++it)
        {
            if (it.getDepth() == m_parameters->precomputation_grid_depth &&
                grid_elements.find(it.getKey()) == grid_elements.end())
            {
                grid_elements.insert(it.getKey());
            }
        }

        for (const auto& key : grid_elements)
        {
            point3d grid_point = m_color_octree->keyToCoord(key, m_parameters->depth);

            grid_point.z() = m_parameters->ground_level;

            OcTreeKey grid_key;

            m_color_octree->coordToKeyChecked(grid_point, m_parameters->depth, grid_key);

            if (m_ground_zone_elements.find(grid_key) != m_ground_zone_elements.end())
            {
                m_grid_elements.insert(grid_key);
            }
        }
    }



    bool AugmentedModel::isInsideBoundaries(const point3d& point, int step) const
    {
        const float resolution = m_parameters->resolution;

        bool inside = false;

        const double coefficients[3] { 11, 35, 2 };

        switch (step)
        {
            case 0:
            {   
                if ((point.x() - coefficients[0] * resolution) > m_model_size.min_x)
                {
                    for (int i = 1; i < coefficients[1]; ++i)
                    {
                        if (isElementOfGroundLevel(point.x() - i * resolution, point.y()))
                        {
                            inside = true;
                            break;
                        }
                    }
                }   
                break;
            }
            case 1:
            {   
                if ((point.x() + coefficients[0] * resolution) < m_model_size.max_x)
                {
                    for (int i = 1; i < coefficients[1]; ++i)
                    {
                        if (isElementOfGroundLevel(point.x() + i * resolution, point.y()))
                        {
                            inside = true;
                            break;
                        }
                    }
                }   
                break;
            }
            case 2:
            {                
                if ((point.y() - coefficients[0] * resolution) > m_model_size.min_y)
                {
                    for (int i = 1; i < coefficients[1]; ++i)
                    {
                        if (isElementOfGroundLevel(point.x(), point.y() - i * resolution))
                        {
                            inside = true;
                            break;
                        }
                    }
                }
                break;
            }
            case 3:
            {                
                if ((point.y() + coefficients[0] * resolution) < m_model_size.max_y)
                {
                    for (int i = 1; i < coefficients[1]; ++i)
                    {
                        if (isElementOfGroundLevel(point.x(), point.y() + i * resolution))
                        {
                            inside = true;
                            break;
                        }
                    }
                }
                break;
            }
            case 4:
            {
                inside = ((point.z() - coefficients[3] * resolution) > 
                    m_model_size.min_z) ? true : false;
                break;
            }
            case 5:
            {
                inside = ((point.z() + coefficients[3] * resolution) < 
                    m_model_size.max_z) ? true : false;
                break;
            }
            default:
            {      
                break;
            }
        }

        return inside;
    }   


    bool AugmentedModel::isElementOfGroundLevel(double x, double y) const
    {
        const point3d point(x, y, m_parameters->ground_level);

        if (OcTreeKey key; m_color_octree->coordToKeyChecked(point, m_parameters->depth, key))
        {
            if (isGroundLevelElement(key))
            {
                return true;
            }
        }

        return false;
    }


    bool AugmentedModel::isGroundLevelElement(const OcTreeKey& key) const
    {
        if (m_ground_level_elements.find(key) != m_ground_level_elements.end())
        {
            return true;
        }
        return false;
    }


    bool AugmentedModel::isGroundZoneElement(const OcTreeKey& key) const
    {
        if (m_ground_zone_elements.find(key) != m_ground_zone_elements.end())
        {
            return true;
        }
        return false;
    }
   
   
    bool AugmentedModel::isReachableElement(const OcTreeKey& key) const
    {
        if (m_reachable_elements.find(key) != m_reachable_elements.end())
        {
            return true;
        }
        return false;
    }
    
    
    bool AugmentedModel::isVerticalElement(const OcTreeKey& key) const
    {
        if (m_vertical_elements.find(key) != m_vertical_elements.end())
        {
            return true;
        }
        return false;
    }
    
    
    bool AugmentedModel::isHorizontalElement(const OcTreeKey& key) const
    {
        if (m_horizontal_elements.find(key) != m_horizontal_elements.end())
        {
            return true;
        }
        return false;
    }
    
    
    bool AugmentedModel::isGeneralElement(const OcTreeKey& key) const
    {
        if (m_general_elements.find(key) != m_general_elements.end())
        {
            return true;
        }
        return false;
    }



    void AugmentedModel::logResults() const
    {
        cout << "Elements:\t\t"            << m_color_octree->size()         << '\n'
             << "Ground level elements:\t" << m_ground_level_elements.size() << '\n'
             << "Ground zone elements:\t"  << m_ground_zone_elements.size()  << '\n'
             << "Reachable elements:\t"    << m_reachable_elements.size()    << '\n'
             << "Vertical elements:\t"     << m_vertical_elements.size()     << '\n'
             << "Horizonal elements:\t"    << m_horizontal_elements.size()   << '\n'
             << "General elements:\t"      << m_general_elements.size()      << '\n'
             << "Grid elements:\t\t"       << m_grid_elements.size()         << '\n';
    }



    bool AugmentedModel::castRay(const point3d& origin, const point3d& direction, 
        const point3d& target, point3d& end, bool ignore_unknown, 
        double max_range, int depth, double resolution) const 
    {
        // Initialization phase -------------------------------------------------------

        OcTreeKey current_key;

        if (m_color_octree->coordToKeyChecked(origin, depth, current_key))
        {
            cout << "Coordinates out of bounds during ray casting" << endl;
            return false;
        }

        NodePtr starting_node = m_color_octree->search(current_key, depth);

        if (starting_node)
        {
            if (m_color_octree->isNodeOccupied(starting_node))
            {
                end = m_color_octree->keyToCoord(current_key, depth);
                return true;
            }
        } 
        else if (!ignore_unknown)
        {
            end = m_color_octree->keyToCoord(current_key, depth);
            return false;
        }

        point3d normalized_direction = direction.normalized();

        bool max_range_set = (max_range > 0.0);

        int step[3];
        double t_max[3];
        double t_delta[3];

        for (unsigned int i = 0; i < 3; ++i)
        {
            if (normalized_direction(i) > 0.0)
            {
                step[i] = 1;
            }
            else if (normalized_direction(i) < 0.0)
            {
                step[i] = -1;
            }
            else
            {
                step[i] = 0;
            }

            if (step[i] != 0)
            {
                double voxel_border = m_color_octree->keyToCoord(current_key[i], depth);
                voxel_border += static_cast<double>(step[i] * resolution * 0.5);

                t_max[i] = (voxel_border - origin(i)) / normalized_direction(i);
                t_delta[i] = resolution / fabs(normalized_direction(i));
            }
            else
            {
                t_max[i] = numeric_limits<double>::max();
                t_delta[i] = numeric_limits<double>::max();
            }
        }

        if (step[0] == 0 && step[1] == 0 && step[2] == 0)
        {
            cout << "Raycasting in direction (0,0,0) is not possible!" << endl;
            return false;
        }

        double max_range_sq = max_range * max_range;

        // Incremental phase  ---------------------------------------------------------

        bool done = false;

        while (!done)
        {
            unsigned int dim;

            if (t_max[0] < t_max[1])
            {
                dim = (t_max[0] < t_max[2]) ? 0 : 2;
            }
            else
            {
                dim = (t_max[1] < t_max[2]) ? 1 : 2;
            }

            // NOTE: Not important if only existing elements are targeted.
            // if ((step[dim] < 0 && current_key[dim] == 0) ||
            //     (step[dim] > 0 && current_key[dim] == 2 * m_color_octree->tree_max_val - 1))    
            // {
            //     cout << "Coordinate hit bounds in dim" << dim << ", aborting raycast" << endl;
            //     end = m_color_octree->keyToCoord(current_key, depth);
            //     return false; 
            // }

            current_key[dim] += step[dim];
            t_max[dim] += t_delta[dim];

            end = m_color_octree->keyToCoord(current_key, depth);

            if (max_range_set)
            {
                double distance_from_origin_sq = 0.0;

                for (unsigned int j = 0; j < 3; ++j)
                { 
                    distance_from_origin_sq += ((end(j) - origin(j)) * (end(j) * origin(j)));
                }

                if (distance_from_origin_sq > max_range_sq)
                {
                    return false;
                }
            }

            NodePtr current_node = m_color_octree->search(current_key, depth);

            if (current_node)
            {
                if (m_color_octree->isNodeOccupied(current_node))
                {
                    done = true;
                    break;
                }
                else if (!ignore_unknown)
                {
                    return false;
                }
            }
            else
            {
                if (evaluateRayCast(target, end, depth))
                {
                    done = true;
                    break;
                }
            }
        }

        return true;
    }


    bool AugmentedModel::evaluateRayCast(const point3d& target_point, 
        const point3d& end_point, int depth) const
    {
        OcTreeKey target_key, end_key;

        if (m_color_octree->coordToKeyChecked(target_point, depth, target_key) &&
            m_color_octree->coordToKeyChecked(end_point, depth, end_key))
        {
            return (target_key == end_key);
        }

        return false;
    }


    bool AugmentedModel::checkRayCast(bool good, const point3d& target_point,
        const point3d& origin, const point3d& direction, point3d& end_point, 
        double max_range, double resolution, bool ignore_unknown) const
    {
        if (castRay(origin, direction, target_point, end_point, true, 
            max_range, m_parameters->depth, resolution))
        {
            if (!good && !evaluateRayCast(target_point, end_point) ||
                 good &&  evaluateRayCast(target_point, end_point))
            {
                return true;
            }
        }

        return false;
    }


    bool AugmentedModel::evaluateRayCast(const point3d& target_point, const point3d& end_point) const
    {
        OcTreeKey target_key, end_key;

        coordToKeyChecked(target_point, m_parameters->depth, target_key);
        coordToKeyChecked(end_point, m_parameters->depth, end_key);

        return (target_key == end_key);
    }



    bool AugmentedModel::coordToKeyChecked(
        const point3d& point, int depth, OcTreeKey& key) const
    {
        return m_color_octree->coordToKeyChecked(point, depth, key);
    }

    
    point3d AugmentedModel::keyToCoord(const OcTreeKey& key, int depth) const
    {
        return m_color_octree->keyToCoord(key, depth);
    }


    void AugmentedModel::createNode(const point3d& point, const Color& color)
    {
        NodePtr node = m_color_octree->search(point, m_parameters->depth);

        if (node != nullptr)
        {
            m_color_octree->updateNode(point, true);
            node = m_color_octree->search(point, m_parameters->depth);
        }

        if (node != nullptr)
        {
            m_color_octree->expandNode(node);
            m_color_octree->pruneNode(node);

            node->setColor(color);
        }
    }


}

