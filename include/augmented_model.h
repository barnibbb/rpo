#pragma once
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <fstream>
#include <iostream>

#include <octomap/ColorOcTree.h>
#include <pcl/features/normal_3d.h>

#include "parameters.h"

using namespace std;

namespace rpo
{
    using octomap::ColorOcTree;
    using octomap::OcTreeKey;
    using octomap::point3d;
    using octomap::AbstractOcTree;

    using NodePtr = octomap::ColorOcTreeNode*;
    using Color = octomap::ColorOcTreeNode::Color;
    using KeySet = unordered_set<OcTreeKey, OcTreeKey::KeyHash>;
    using SurfaceNormals = unordered_map<OcTreeKey, point3d, OcTreeKey::KeyHash>;

    struct ModelSize { double min_x, min_y, min_z, max_x, max_y, max_z; };

    class AugmentedModel
    {
    public:

        // Methods ------------------------------------------------------------------

        AugmentedModel(const Parameters& parameters);
        
        void deleteCeiling();
        void expandModel();
        void smoothModel();
        
        void computeGroundLevel();
        void computeGroundZone();
        void computeReachableElements();
        void computeNormals();
        void computePlanes();
        void computeGridElements();

        bool isInsideBoundaries(const point3d& point, int step) const;
        bool isElementOfGroundLevel(double x, double y) const;

        bool isGroundLevelElement(const OcTreeKey& key) const;
        bool isGroundZoneElement(const OcTreeKey& key) const;
        bool isReachableElement(const OcTreeKey& key) const;
        bool isVerticalElement(const OcTreeKey& key) const;
        bool isHorizontalElement(const OcTreeKey& key) const;
        bool isGeneralElement(const OcTreeKey& key) const;


        void logResults() const;
        

        // Improved ray cast to consider arbitrary depth and resolution
        bool castRay(const point3d& origin, const point3d& direction, 
            const point3d& target, point3d& end, bool ignore_unknown, 
            double max_range, int depth, double resolution) const;

        // Ray cast to consider hitting a target without marking it occupied
        bool evaluateRayCast(const point3d& target_point, 
            const point3d& end_point, int depth) const;

        bool checkRayCast(bool good, const point3d& target_point,
            const point3d& origin, const point3d& direction, point3d& end_point, 
            double max_range, double resolution, bool ignore_unknown) const;

        bool evaluateRayCast(const point3d& target_point, const point3d& end_point) const;
        

        bool coordToKeyChecked(const point3d& point, int depth, OcTreeKey& key) const;

        point3d keyToCoord(const OcTreeKey& key, int depth) const;

        void createNode(const point3d& point, const Color& color);



        // Data memebers -------------------------------------------------------------
    
        Parameters m_parameters;

        shared_ptr<ColorOcTree> m_color_octree = nullptr;

        KeySet m_ground_level_elements;   
        KeySet m_ground_zone_elements;    
        KeySet m_reachable_elements;      
        KeySet m_vertical_elements;
        KeySet m_horizontal_elements;
        KeySet m_general_elements;
        KeySet m_grid_elements;           

        SurfaceNormals m_surface_normals;

        ModelSize m_model_size;
    };
}
