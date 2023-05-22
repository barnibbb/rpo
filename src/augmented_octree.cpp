#include "augmented_octree.h"

using namespace octomap;

namespace rpo
{
    using NodePtr = AugmentedOcTreeNode*;

    void AugmentedOcTreeNode::updateDoseChildren()
    {
        m_dose = getAverageChildDose();
    }

    double AugmentedOcTreeNode::getAverageChildDose() const 
    {
        double sum = 0;
        double num = 0;

        if (children != NULL)
        {
            for (int i = 0; i < 8; i++)
            {
                AugmentedOcTreeNode* child = static_cast<AugmentedOcTreeNode*>(children[i]);

                if (child != NULL && child->isDoseSet()) 
                {
                    sum += child->getDose();
                    num += 1;
                }
            }
        }

        if (num > 0)
        {
            return sum / num;
        }
        else
        {
            return 0;
        }
    }

    double AugmentedOcTreeNode::getSumChildDose() const
    {
        double sum = 0;

        if (children != NULL)
        {
            for (int i = 0; i < 8; i++)
            {
                AugmentedOcTreeNode* child = static_cast<AugmentedOcTreeNode*>(children[i]);

                if (child != NULL && child->isDoseSet()) 
                {
                    sum += child->getDose();
                }
            }
        }

        return sum;
    }

    std::istream& AugmentedOcTreeNode::readData(std::istream &s)
    {
        s.read((char*) &value, sizeof(value));
        s.read((char*) &color, sizeof(Color));
        s.read((char*) &m_dose, sizeof(m_dose));

        return s;
    }

    std::ostream& AugmentedOcTreeNode::writeData(std::ostream &s) const
    {
        s.write((const char*) &value, sizeof(value));
        s.write((const char*) &color, sizeof(Color));
        s.write((const char*) &m_dose, sizeof(m_dose));

        return s;
    }

    // Octomap related methods --------------------------------------------------------------

    AugmentedOcTree::AugmentedOcTree(double resolution) : OccupancyOcTreeBase<AugmentedOcTreeNode>(resolution)
    {
        m_augmented_octree_member_init.ensureLinking();
    }

    bool AugmentedOcTree::pruneNode(AugmentedOcTreeNode* node)
    {
        if (!isNodeCollapsible(node))
        {
            return false;
        }

        node->copyData(*(getNodeChild(node, 0)));

        if (node->isColorSet() && node->isDoseSet())
        {
            node->setColor(node->getAverageChildColor());
            node->setDose(node->getSumChildDose());
        }

        for (unsigned int i = 0; i < 8; ++i)
        {
            deleteNodeChild(node, i);
        }

        delete[] node->children;
        node->children = NULL;

        return true;
    }

    void AugmentedOcTree::expandNode(AugmentedOcTreeNode* node)
    {
        assert(!nodeHasChildren(node));

        double parent_dose = node->getDose();

        for (unsigned int i = 0; i < 8; ++i)
        {
            AugmentedOcTreeNode* child = createNodeChild(node, i);
            child->copyData(*node);
            child->setDose(parent_dose / 8);
            child->setColor(node->getColor());
        }
    }

    bool AugmentedOcTree::isNodeCollapsible(const AugmentedOcTreeNode* node) const
    {
        if (!nodeChildExists(node, 0))
        {
            return false;
        }

        const AugmentedOcTreeNode* first_child = getNodeChild(node, 0);

        if (nodeHasChildren(first_child))
        {
            return false;
        }

        for (unsigned int i = 1; i < 8; ++i)
        {
            if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || 
                !(getNodeChild(node, i)->getValue() == first_child->getValue()))
            {
                return false;
            }
        }

        return true;
    }

    AugmentedOcTreeNode* AugmentedOcTree::setNodeColor(const OcTreeKey& key, uint8_t r, uint8_t g, uint8_t b)
    {
        AugmentedOcTreeNode* node = search(key);

        if (node != 0)
        {
            node->setColor(r, g, b);
        }

        return node;
    }

    AugmentedOcTreeNode* AugmentedOcTree::setNodeColor(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b)
    {
        OcTreeKey key;

        if (!this->coordToKeyChecked(point3d(x, y, z), key))
        {
            return NULL;
        }

        return setNodeColor(key, r, g, b);
    }

    AugmentedOcTreeNode* AugmentedOcTree::setNodeDose(const OcTreeKey& key, double dose)
    {
        AugmentedOcTreeNode* node = search(key);

        if (node != 0)
        {
            node->setDose(dose);
        }

        return node;
    }

    AugmentedOcTreeNode* AugmentedOcTree::setNodeDose(float x, float y, float z, double dose)
    {
        OcTreeKey key;

        if (!this->coordToKeyChecked(point3d(x, y, z), key))
        {
            return NULL;
        }

        return setNodeDose(key, dose);
    }

    AugmentedOcTreeNode* AugmentedOcTree::averageNodeColor(const OcTreeKey& key, uint8_t r, uint8_t g, uint8_t b)
    {
        AugmentedOcTreeNode* node = search(key);

        if (node != 0)
        {
            if (node->isColorSet())
            {
                AugmentedOcTreeNode::Color prev_color = node->getColor();
                node->setColor((prev_color.r + r) / 2, (prev_color.g + g) / 2, (prev_color.b + b) / 2);
            }
            else
            {
                node->setColor(r, g, b);
            }
        }

        return node;
    }

    AugmentedOcTreeNode* AugmentedOcTree::averageNodeColor(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b)
    {
        OcTreeKey key;

        if (!this->coordToKeyChecked(point3d(x, y, z), key))
        {
            return NULL;
        }

        return averageNodeColor(key, r, g, b);
    }

    AugmentedOcTreeNode* AugmentedOcTree::incrementNodeDose(const OcTreeKey& key, double dose)
    {
        AugmentedOcTreeNode* node = search(key);

        if (node != 0)
        {
            if (node->isDoseSet())
            {
                float prev_dose = node->getDose();
                node->setDose(prev_dose + dose);
            }
            else
            {
                node->setDose(dose);
            }
        }

        return node;
    }

    AugmentedOcTreeNode* AugmentedOcTree::incrementNodeDose(float x, float y, float z, double dose)
    {
        OcTreeKey key;

        if (!this->coordToKeyChecked(point3d(x, y, z), key))
        {
            return NULL;
        }

        return incrementNodeDose(key, dose);
    }

    void AugmentedOcTree::updateInnerOccupancy()
    {
        this->updateInnerOccupancyRecurs(this->root, 0);
    }

    void AugmentedOcTree::updateInnerOccupancyRecurs(AugmentedOcTreeNode* node, unsigned int depth)
    {
        if (nodeHasChildren(node))
        {
            if (depth < this->tree_depth)
            {
                for (unsigned int i = 0; i < 8; ++i)
                {
                    if (nodeChildExists(node, i))
                    {
                        updateInnerOccupancyRecurs(getNodeChild(node, i), depth + 1);
                    }
                }
            }
            node->updateOccupancyChildren();
            node->updateColorChildren();
            node->updateDoseChildren();
        }
    }

    std::ostream& operator<<(std::ostream& out, AugmentedOcTreeNode::Color const& c)
    {
        return out << '(' << (unsigned int)c.r << ' ' << (unsigned int)c.g << ' ' << (unsigned int)c.b << ')';
    }

    AugmentedOcTree::StaticMemberInitializer AugmentedOcTree::m_augmented_octree_member_init;


    // RPO related methods ------------------------------------------------------------------

    void AugmentedOcTree::setParameters(const Parameters& parameters)
    {
        m_parameters = parameters;
    }

    Parameters AugmentedOcTree::getParameters() const
    {
        return this->m_parameters;
    }


    void AugmentedOcTree::modifyModel()
    {
        for (leaf_iterator it = this->begin(), end = this->end(); it != end; ++it)
        {
            if (it.getDepth() < m_parameters.depth)
            {
                NodePtr node = this->search(it.getKey(), it.getDepth());

                if (node != nullptr)
                {
                    this->expandNode(node);
                }
            }
        }

        if (m_parameters.delete_ceiling)
        {
            deleteCeiling();
        }

        if (m_parameters.smooth_model)
        {
            smoothModel();
        }

        if (m_parameters.expand_model)
        {
            expandModel();
        }
    }

    void AugmentedOcTree::deleteCeiling()
    {
        KeySet elements_to_delete;
        
        for (leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it)
        {
            if (it.getCoordinate().z() >= this->max_value[2] - 0.4)
            {
                elements_to_delete.insert(it.getKey());
            }
        }

        for (const auto& key : elements_to_delete)
        {
            this->deleteNode(key, m_parameters.depth);
        }
    }

    void AugmentedOcTree::smoothModel()
    {
        const float resolution = m_parameters.resolution;

        const std::vector<point3d> steps 
        {
            { -resolution, 0, 0 }, { resolution, 0, 0 }, 
            { 0, -resolution, 0 }, { 0, resolution, 0 }
        };

        KeySet elements_to_delete, checked_elements;

        for (leaf_iterator it = this->begin(), end = this->end(); it != end; ++it)
        {
            KeySet elements_of_plane, unchecked_elements, elements_to_check;

            elements_of_plane.insert(it.getKey());

            unchecked_elements.insert(it.getKey());

            while (unchecked_elements.size() > 0)
            {
                for (const auto& element : unchecked_elements)
                {
                    checked_elements.insert(element);

                    point3d center = this->keyToCoord(element);

                    for (const auto& step : steps)
                    {
                        point3d neighbor = center + step;

                        OcTreeKey neighbor_key;

                        this->coordToKeyChecked(neighbor, m_parameters.depth, neighbor_key);

                        if (NodePtr node = this->search(neighbor_key, m_parameters.depth); node != nullptr)
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
                    point3d lower = this->keyToCoord(element);

                    point3d middle = lower + point3d(0, 0, resolution);

                    point3d upper = middle + point3d(0, 0, resolution);

                    if (NodePtr middle_node = this->search(middle, m_parameters.depth); middle_node != nullptr)
                    {
                        if (NodePtr upper_node = this->search(upper, m_parameters.depth); upper_node == nullptr)
                        {
                            bool ok = true;

                            for (const auto& step : steps)
                            {
                                point3d neighbor = middle + step;

                                if (NodePtr n_node = this->search(neighbor, m_parameters.depth); n_node == nullptr)
                                {
                                    ok = false;
                                    break;
                                }
                            }

                            if (!ok)
                            {
                                OcTreeKey middle_key;

                                this->coordToKeyChecked(middle, m_parameters.depth, middle_key);

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
            NodePtr node = this->search(element, m_parameters.depth);
            
            if (node != nullptr)
            {
                this->deleteNode(element, m_parameters.depth);
            }
        }
    }

    void AugmentedOcTree::expandModel()
    {
        m_parameters.depth += 1;
        m_parameters.resolution /= 2;

        this->expand();

        for (leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it)
        {
            NodePtr node = this->search(it.getKey(), m_parameters.depth - 1);

            if (node != nullptr)
            {
                this->expandNode(node);
            }
        }
    }

    void AugmentedOcTree::computeGroundLevel()
    {
        std::map<double, int> height_map;

        // First it is counted how many voxels belong to each height.
        for (leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it)
        {
            const point3d point = it.getCoordinate();

            if (NodePtr node = this->search(point, m_parameters.depth); node != nullptr)
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
            if (height.second > 0.05 * this->getNumLeafNodes())
            {
                ++level_counter;
            }

            if ((!m_parameters.expand_model && level_counter == 1) || 
                 (m_parameters.expand_model && level_counter == 2))
            {
                m_parameters.ground_level = height.first;

                break;
            }
        }
    }

    void AugmentedOcTree::computeGroundZone()
    {
        const float resolution = m_parameters.resolution;

        KeySet ground_zone;

        for (leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it)
        {
            point3d point = it.getCoordinate();

            if (point.z() == m_parameters.ground_level)
            {
                if (NodePtr node = this->search(it.getKey()); node != nullptr)
                {
                    node->setVoxelType(VoxelType::Ground);
                }

                point.z() += resolution;

                bool ground = true;

                const double lamp_top = 
                    m_parameters.ground_level + 
                    m_parameters.resolution   +
                    m_parameters.lamp_offset  + 
                    m_parameters.lamp_height;

                while (point.z() < lamp_top)
                {
                    NodePtr node = this->search(point);

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
            const point3d center = this->keyToCoord(key, m_parameters.depth);

            bool ground = true;

            // For each possible ground zone element it checked whether its
            // neighbors are also ground zone elements. If all neighbors in
            // a given region also belong to the ground zone, then the center
            // is considered as real ground zone element.
            int min_dist = static_cast<int>(
                m_parameters.min_distance_from_obstacles / m_parameters.resolution);

            for (int i = -min_dist; i <= min_dist; ++i)
            {
                for (int j = -min_dist; j <= min_dist; ++j)
                {
                    const point3d neighbor = center + 
                        point3d(i * resolution, j * resolution, 0);

                    if (OcTreeKey neighbor_key; this->coordToKeyChecked(
                        neighbor, m_parameters.depth, neighbor_key))
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
                if (NodePtr node = this->search(key); node != nullptr)
                {
                    node->setGroundZoneValue(true);
                }
            }
        }
    }

    void AugmentedOcTree::computeReachableElements()
    {
        const float resolution = m_parameters.resolution;

        const std::vector<point3d> steps {
            { -resolution, 0, 0 }, { resolution, 0, 0 }, 
            { 0, -resolution, 0 }, { 0, resolution, 0 }, 
            { 0, 0, -resolution }, { 0, 0, resolution } };

        const std::vector<int> inverse_index { 1, 0, 3, 2, 5, 4 };

        KeySet unchecked_points, checked_points;

        point3d start_point(
            m_parameters.reachable_region_x,
            m_parameters.reachable_region_y, 
            m_parameters.ground_level + m_parameters.resolution / 2.0 + 
            m_parameters.lamp_offset + m_parameters.lamp_height / 2.0);

        OcTreeKey start_key;

        this->coordToKeyChecked(start_point, m_parameters.depth, start_key);

        unchecked_points.insert(start_key);

        checked_points.insert(start_key);

        while (unchecked_points.size() > 0)
        {
            KeySet points_to_add;

            for (const auto& center_key : unchecked_points)
            {
                point3d center_point =  this->keyToCoord(center_key, m_parameters.depth);

                for (int s = 0; s < steps.size(); ++s)
                {
                    if (!(center_point.z() < m_parameters.lamp_offset && s == 5))
                    {
                        point3d neighbor_point = center_point + steps[s];

                        if (neighbor_point.x() >= this->min_value[0] && neighbor_point.x() <= this->max_value[0] &&
                            neighbor_point.y() >= this->min_value[1] && neighbor_point.y() <= this->max_value[1] &&
                            neighbor_point.z() >= this->min_value[2] && neighbor_point.z() <= this->max_value[2])
                        {
                            OcTreeKey neighbor_key;

                            this->coordToKeyChecked(neighbor_point, m_parameters.depth, neighbor_key);

                            if (checked_points.find(neighbor_key) == checked_points.end())
                            {
                                if (NodePtr node = this->search(neighbor_key, m_parameters.depth); node == nullptr)
                                {
                                    if (points_to_add.find(neighbor_key) == points_to_add.end()) 
                                    {
                                        points_to_add.insert(neighbor_key);

                                        checked_points.insert(neighbor_key);
                                    }
                                }
                                else
                                {
                                    if (isInsideBoundaries(neighbor_point, inverse_index[s]))
                                    {
                                        node->setReachability(true);

                                        if (m_reachable_elements.find(neighbor_key) == m_reachable_elements.end())
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

    void AugmentedOcTree::computeNormals()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr octree_points (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr octree_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>());

        for (leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it)
        {
            const point3d point = it.getCoordinate();

            octree_points->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
        }

        kdtree->setInputCloud(octree_points);

        normal_estimation.setInputCloud(octree_points);
        normal_estimation.setSearchMethod(kdtree);
        normal_estimation.setRadiusSearch(2 * m_parameters.resolution);
        normal_estimation.compute(*octree_normals);

        for (size_t i = 0; i < octree_points->size(); ++i)
        {
            const pcl::PointXYZ pcl_point = octree_points->points[i];

            const point3d point(pcl_point.data[0], pcl_point.data[1], pcl_point.data[2]);

            if (NodePtr node = this->search(point, m_parameters.depth); node != nullptr)
            {
                const pcl::Normal normal = octree_normals->at(i);

                node->setNormal(point3d(normal.normal_x, normal.normal_y, normal.normal_z));
            }
        }
    }

    void AugmentedOcTree::computePlanes()
    {
        const float resolution = m_parameters.resolution;

        const int depth = m_parameters.depth;

        // Modifies surface normals on horizontal surfaces
        if (m_parameters.modify_normals)
        {
            for (const auto& element : m_reachable_elements)
            {
                point3d point = this->keyToCoord(element, depth);

                if (NodePtr node = this->search(point + point3d(0, 0, resolution), depth); node == nullptr)
                {
                    bool plane_element = true;

                    for (int x = -1; x < 2; ++x)
                    {
                        for (int y = -1; y < 2; ++y)
                        {
                            if (NodePtr neighbor_node = this->search(point + 
                                point3d(x * resolution,y * resolution, 0), depth);
                                neighbor_node == nullptr)
                            {
                                plane_element = false;
                            }
                        }
                    }

                    if (plane_element)
                    {
                        if (NodePtr node_normal = this->search(element, depth); node_normal != nullptr)
                        {
                            node_normal->setNormal(point3d(0, 0, 1));
                        }
                    }
                }
            }

            for (const auto& element : m_reachable_elements)
            {
                point3d point = this->keyToCoord(element, depth);

                if (NodePtr node = this->search(element, depth); node != nullptr)
                {
                    point3d normal = node->getNormal();

                    if (normal.x() > 0)
                    {
                        if (NodePtr neighbor_node = this->search(
                            point + point3d(resolution, 0, 0), depth); neighbor_node != nullptr)
                        {
                            node->setNormal(point3d(-normal.x(), normal.y(), normal.z()));
                        }
                    }
                    else if (normal.x() < 0)
                    {
                        if (NodePtr neighbor_node = this->search(
                            point + point3d(-resolution, 0, 0), depth); neighbor_node != nullptr)
                        {
                            node->setNormal(point3d(-normal.x(), normal.y(), normal.z()));
                        }
                    }

                    normal = node->getNormal();

                    if (normal.y() > 0)
                    {
                        if (NodePtr neighbor_node = this->search(
                            point + point3d(0, resolution, 0), depth); neighbor_node != nullptr)
                        {
                            node->setNormal(point3d(normal.x(), -normal.y(), normal.z()));
                        }
                    }
                    else if (normal.y() < 0)
                    {
                        if (NodePtr neighbor_node = this->search(
                            point + point3d(0, -resolution, 0), depth); neighbor_node != nullptr)
                        {
                            node->setNormal(point3d(normal.x(), -normal.y(), normal.z()));
                        }
                    }

                    normal = node->getNormal();

                    if (normal.z() > 0)
                    {
                        if (NodePtr neighbor_node = this->search(
                            point + point3d(0, 0, resolution), depth); neighbor_node != nullptr)
                        {
                            node->setNormal(point3d(normal.x(), normal.y(), -normal.z()));
                        }
                    }
                    else if (normal.z() < 0)
                    {
                        if (NodePtr neighbor_node = this->search(
                            point + point3d(0, 0, -resolution), depth); neighbor_node != nullptr)
                        {
                            node->setNormal(point3d(normal.x(), normal.y(), -normal.z()));
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

            NodePtr node = this->search(element, depth);

            const double angle_to_vertical = abs(node->getNormal().angleTo(vertical));
                
            point3d point = this->keyToCoord(element, depth);

            if ((angle_to_vertical < M_PI / 18.0 || angle_to_vertical > (M_PI * 17.0 / 18.0)) && point.z() > m_parameters.ground_level)
            {
                node->setVoxelType(VoxelType::Horizontal);
            }
            else if (angle_to_vertical > (quarter - M_PI / 18.0) && angle_to_vertical < (quarter + M_PI / 18.0))
            {
                node->setVoxelType(VoxelType::Vertical);
            }             
        }
    }

    void AugmentedOcTree::computeGridElements()
    {
        KeySet grid_elements;

        for (tree_iterator it = this->begin_tree(), end = this->end_tree(); it != end; ++it)
        {
            if (it.getDepth() == m_parameters.precomputation_grid_depth &&
                grid_elements.find(it.getKey()) == grid_elements.end())
            {
                grid_elements.insert(it.getKey());
            }
        }

        for (const auto& key : grid_elements)
        {
            point3d grid_point = this->keyToCoord(key, m_parameters.depth);

            grid_point.z() = m_parameters.ground_level;

            if (NodePtr node = this->search(grid_point, m_parameters.depth))
            {
                node->setGridValue(true);
            }
        }
    }


    bool AugmentedOcTree::isGroundLevelElement(const OcTreeKey& key) const
    {
        if (NodePtr node = this->search(key, m_parameters.depth); node != nullptr)
        {
            if (node->getVoxelType() == VoxelType::Ground)
            {
                return true;
            }
        }

        return false;
    }

    bool AugmentedOcTree::isVerticalElement(const OcTreeKey& key) const
    {
        if (NodePtr node = this->search(key, m_parameters.depth); node != nullptr)
        {
            if (node->getVoxelType() == VoxelType::Vertical)
            {
                return true;
            }
        }

        return false;
    }

    bool AugmentedOcTree::isHorizontalElement(const OcTreeKey& key) const
    {
        if (NodePtr node = this->search(key, m_parameters.depth); node != nullptr)
        {
            if (node->getVoxelType() == VoxelType::Horizontal)
            {
                return true;
            }
        }

        return false;
    }

    bool AugmentedOcTree::isGeneralElement(const OcTreeKey& key) const
    {
        if (NodePtr node = this->search(key, m_parameters.depth); node != nullptr)
        {
            if (node->getVoxelType() == VoxelType::General)
            {
                return true;
            }
        }

        return false;
    }

    bool AugmentedOcTree::isGroundZoneElement(const OcTreeKey& key) const
    {
        if (NodePtr node = this->search(key, m_parameters.depth); node != nullptr)
        {
            return node->isGroundZone();
        }

        return false;
    }

    bool AugmentedOcTree::isReachableElement(const OcTreeKey& key) const
    {
        if (NodePtr node = this->search(key, m_parameters.depth); node != nullptr)
        {
            return node->isReachable();
        }

        return false;
    }

    bool AugmentedOcTree::isGridElement(const OcTreeKey& key) const
    {
        if (NodePtr node = this->search(key, m_parameters.depth); node != nullptr)
        {
            return node->isGrid();
        }

        return false;
    }



    bool AugmentedOcTree::isInsideBoundaries(const point3d& point, int step) const
    {
        const float resolution = m_parameters.resolution;

        bool inside = false;

        const double coefficients[3] { 11, 35, 2 };

        switch (step)
        {
            case 0:
            {   
                if ((point.x() - coefficients[0] * resolution) > this->min_value[0])
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
                if ((point.x() + coefficients[0] * resolution) < this->max_value[0])
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
                if ((point.y() - coefficients[0] * resolution) > this->min_value[1])
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
                if ((point.y() + coefficients[0] * resolution) < this->max_value[1])
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
                inside = ((point.z() - coefficients[3] * resolution) > this->min_value[2]) ? true : false;
                break;
            }
            case 5:
            {
                inside = ((point.z() + coefficients[3] * resolution) < this->max_value[2]) ? true : false;
                break;
            }
            default:
            {      
                break;
            }
        }

        return inside;
    }

    bool AugmentedOcTree::isElementOfGroundLevel(double x, double y) const
    {
        const point3d point(x, y, m_parameters.ground_level);

        if (NodePtr node = this->search(point, m_parameters.depth); node != nullptr)
        {
            if (node->getVoxelType() == VoxelType::Ground)
            {
                return true;
            }
        }

        return false;
    }


    KeySet AugmentedOcTree::getReachableElements() const
    {
        return this->m_reachable_elements;
    }


    // Ray cast related methods -------------------------------------------------------------

    bool AugmentedOcTree::castRay(const point3d& origin, const point3d& direction, 
        const point3d& target, point3d& end, bool ignore_unknown, 
        double max_range, int depth, double resolution) const
    {
        // Initialization phase -------------------------------------------------------

        OcTreeKey current_key;

        if (this->coordToKeyChecked(origin, depth, current_key))
        {
            std::cout << "Coordinates out of bounds during ray casting" << std::endl;
            return false;
        }

        NodePtr starting_node = this->search(current_key, depth);

        if (starting_node)
        {
            if (this->isNodeOccupied(starting_node))
            {
                end = this->keyToCoord(current_key, depth);
                return true;
            }
        } 
        else if (!ignore_unknown)
        {
            end = this->keyToCoord(current_key, depth);
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
                double voxel_border = this->keyToCoord(current_key[i], depth);
                voxel_border += static_cast<double>(step[i] * resolution * 0.5);

                t_max[i] = (voxel_border - origin(i)) / normalized_direction(i);
                t_delta[i] = resolution / std::fabs(normalized_direction(i));
            }
            else
            {
                t_max[i] = std::numeric_limits<double>::max();
                t_delta[i] = std::numeric_limits<double>::max();
            }
        }

        if (step[0] == 0 && step[1] == 0 && step[2] == 0)
        {
            std::cout << "Raycasting in direction (0,0,0) is not possible!" << std::endl;
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

            if ((step[dim] < 0 && current_key[dim] == 0) ||
                (step[dim] > 0 && current_key[dim] == 2 * this->tree_max_val - 1))    
            {
                std::cout << "Coordinate hit bounds in dim" << dim << ", aborting raycast" << std::endl;
                end = this->keyToCoord(current_key, depth);
                return false; 
            }

            current_key[dim] += step[dim];
            t_max[dim] += t_delta[dim];

            end = this->keyToCoord(current_key, depth);

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

            NodePtr current_node = this->search(current_key, depth);

            if (current_node)
            {
                if (this->isNodeOccupied(current_node))
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

    bool AugmentedOcTree::evaluateRayCast(const point3d& target_point, 
        const point3d& end_point, int depth) const
    {
        OcTreeKey target_key, end_key;

        if (this->coordToKeyChecked(target_point, depth, target_key) &&
            this->coordToKeyChecked(end_point, depth, end_key))
        {
            return (target_key == end_key);
        }

        return false;
    }

    bool AugmentedOcTree::checkRayCast(bool good, const point3d& target_point,
        const point3d& origin, const point3d& direction, point3d& end_point, 
        double max_range, int depth, double resolution, bool ignore_unknown) const
    {
        if (castRay(origin, direction, target_point, end_point, ignore_unknown, 
            max_range, depth, resolution))
        {
            if (!good && !evaluateRayCast(target_point, end_point, depth) ||
                 good &&  evaluateRayCast(target_point, end_point, depth))
            {
                return true;
            }
        }

        return false;
    }

}
