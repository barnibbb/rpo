#include <octomap/ColorOcTree.h>

using octomap::ColorOcTree;
using octomap::OcTreeKey;
using octomap::point3d;
using octomap::ColorOcTreeNode;

namespace rpo {

    class CustomOcTree : public ColorOcTree {
    public:
        // Improved ray cast to consider custom depth and resolution
        bool castRay(const point3d& origin, const point3d& direction, 
            const point3d& target, point3d& end, bool ignore_unknown, 
            double max_range, int depth, double resolution) const;

        // Evaluation function for ray cast
        bool evaluateRayCast(const point3d& target_point, 
            const point3d& end_point, int depth) const;
    };
}



