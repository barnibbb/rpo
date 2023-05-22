#include <string>
#include <fstream>
#include <iostream>
#include <memory>
#include <unordered_set>
#include <map>

#include <octomap/ColorOcTree.h>
#include <pcl/features/normal_3d.h>

#include "parameters.h"

using namespace octomap;

namespace rpo
{
    using KeySet = unordered_set<OcTreeKey, OcTreeKey::KeyHash>;

    enum class VoxelType { Ground, Vertical, Horizontal, General };

    class AugmentedOcTree;

    class AugmentedOcTreeNode : public ColorOcTreeNode
    {
    public:
        friend class AugmentedOcTree;
            
        AugmentedOcTreeNode() : ColorOcTreeNode(), m_dose(0) {}
        AugmentedOcTreeNode(const AugmentedOcTreeNode& node) : ColorOcTreeNode(node), m_dose(node.m_dose) {}

        bool operator==(const AugmentedOcTreeNode& node) const
        {
        return (node.value == value && node.color == color && node.m_dose == m_dose);
        }

        void copyData(const AugmentedOcTreeNode& from)
        {
        ColorOcTreeNode::copyData(from);
        this->color = from.getColor();
        this->m_dose  = from.getDose();
        }

        inline double getDose() const { return m_dose; }
        inline void setDose(double d) { this->m_dose = d; }

        inline bool isGroundZone() const { return m_ground_zone; }
        inline void setGroundZoneValue(bool ground_zone) { this->m_ground_zone = ground_zone; }
        inline bool isReachable() const { return m_reachable; }
        inline void setReachability(bool reachable) { this->m_reachable = reachable; } 
        inline bool isGrid() const { return m_grid; }
        inline void setGridValue(bool grid) { this->m_grid = grid; }

        inline point3d getNormal() const { return m_normal; }
        inline void setNormal(const point3d& normal) { this->m_normal = normal; }
        inline VoxelType getVoxelType() const { return m_voxel_type; }
        inline void setVoxelType(const VoxelType& voxel_type) { this->m_voxel_type = voxel_type; }

        inline bool isDoseSet() const { return m_dose != 0; }

        void updateDoseChildren();

        double getAverageChildDose() const;
        double getSumChildDose() const;

        std::istream& readData(std::istream& s);
        std::ostream& writeData(std::ostream& s) const;

    protected:
        double m_dose = 0;

        bool m_ground_zone = false;
        bool m_reachable = false;
        bool m_grid = false;

        point3d m_normal = point3d(0, 0, 0);

        VoxelType m_voxel_type = VoxelType::General;
    };


    class AugmentedOcTree : public OccupancyOcTreeBase<AugmentedOcTreeNode>
    {
    public:

        // Octomap related methods --------------------------------------------------------------

        AugmentedOcTree(double resolution);

        AugmentedOcTree* create() const { return new AugmentedOcTree(resolution); }

        std::string getTreeType() const { return "AugmentedOcTree"; }

        virtual bool pruneNode(AugmentedOcTreeNode* node);

        virtual void expandNode(AugmentedOcTreeNode* node);

        virtual bool isNodeCollapsible(const AugmentedOcTreeNode* node) const;

        AugmentedOcTreeNode* setNodeColor(const OcTreeKey& key, uint8_t r, uint8_t g, uint8_t b);

        AugmentedOcTreeNode* setNodeColor(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b);

        AugmentedOcTreeNode* setNodeDose(const OcTreeKey& key, double dose);

        AugmentedOcTreeNode* setNodeDose(float x, float y, float z, double dose);

        AugmentedOcTreeNode* averageNodeColor(const OcTreeKey& key, uint8_t r, uint8_t g, uint8_t b);

        AugmentedOcTreeNode* averageNodeColor(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b);

        AugmentedOcTreeNode* incrementNodeDose(const OcTreeKey& key, double dose);

        AugmentedOcTreeNode* incrementNodeDose(float x, float y, float z, double dose);

        void updateInnerOccupancy();


        // RPO related methods ------------------------------------------------------------------

        void setParameters(const Parameters& parameters);
        Parameters getParameters() const;

        void modifyModel();
        void deleteCeiling();
        void expandModel();
        void smoothModel();

        void computeGroundLevel();
        void computeGroundZone();
        void computeReachableElements();
        void computeNormals();
        void computePlanes();
        void computeGridElements();

        bool isGroundLevelElement(const OcTreeKey& key) const;
        bool isGroundZoneElement(const OcTreeKey& key) const;
        bool isReachableElement(const OcTreeKey& key) const;
        bool isVerticalElement(const OcTreeKey& key) const;
        bool isHorizontalElement(const OcTreeKey& key) const;
        bool isGeneralElement(const OcTreeKey& key) const;
        bool isGridElement(const OcTreeKey& key) const;


        bool isInsideBoundaries(const point3d& point, int step) const;
        bool isElementOfGroundLevel(double x, double y) const;


        KeySet getReachableElements() const;


        // Ray cast related methods -------------------------------------------------------------

        // Improved ray cast to consider arbitrary depth and resolution
        bool castRay(const point3d& origin, const point3d& direction, 
            const point3d& target, point3d& end, bool ignore_unknown, 
            double max_range, int depth, double resolution) const;

        // Ray cast to consider hitting a target without marking it occupied
        bool evaluateRayCast(const point3d& target_point, 
            const point3d& end_point, int depth) const;

        bool checkRayCast(bool good, const point3d& target_point,
            const point3d& origin, const point3d& direction, point3d& end_point, 
            double max_range, int depth, double resolution, bool ignore_unknown) const;


    protected:
        void updateInnerOccupancyRecurs(AugmentedOcTreeNode* node, unsigned int depth);

        class StaticMemberInitializer{
        public:
            StaticMemberInitializer() {
            AugmentedOcTree* tree = new AugmentedOcTree(0.1);
            tree->clearKeyRays();
            AbstractOcTree::registerTreeType(tree);
            }

            void ensureLinking() {};
        };

        static StaticMemberInitializer m_augmented_octree_member_init;

        Parameters m_parameters;

        KeySet m_reachable_elements;
    };

    std::ostream& operator<<(std::ostream& out, AugmentedOcTreeNode::Color const& c);
}

