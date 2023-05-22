#include <memory>

#include "augmented_octree.h"

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

int main (int argc, char** argv)
{
    const std::string file = "/home/barni/rpo_ws/src/rpo/models/hospital_model.ot";

    std::ifstream f (file);

    if (f.is_open())
    {
        std::unique_ptr<octomap::ColorOcTree> color_octree = nullptr;

        color_octree.reset(dynamic_cast<octomap::ColorOcTree*>(octomap::AbstractOcTree::read(f)));

        std::cout << color_octree->getNumLeafNodes() << std::endl;

        std::unique_ptr<rpo::AugmentedOcTree> augmented_octree = std::make_unique<rpo::AugmentedOcTree>(color_octree->getResolution());

        for (octomap::ColorOcTree::leaf_iterator it = color_octree->begin_leafs(), end = color_octree->end_leafs(); it != end; ++it)
        {
            rpo::AugmentedOcTreeNode* node = augmented_octree->search(it.getKey(), it.getDepth());
            
            if (node == NULL)
            {
                node = augmented_octree->updateNode(it.getCoordinate(), true);
                node->setColor(it->getColor());
                node->setDose(0);
                node->setGroundZoneValue(false);
                node->setReachability(false);
                node->setGridValue(false);
                node->setNormal(point3d(0, 0, 0));
                node->setVoxelType(rpo::VoxelType::General);
            }
        }

        std::cout << augmented_octree->getNumLeafNodes() << std::endl;
    }

    return 0;
}
