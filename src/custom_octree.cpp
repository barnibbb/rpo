#include <iostream>

#include "custom_octree.h"

using std::cout;
using std::endl;
using std::numeric_limits;
using std::fabs;

namespace rpo {

    bool CustomOcTree::castRay(const point3d& origin, const point3d& direction, 
        const point3d& target, point3d& end, bool ignore_unknown, 
        double max_range, int depth, double resolution) const {

        // Initialization phase -------------------------------------------------------

        OcTreeKey current_key;

        if (!coordToKeyChecked(origin, depth, current_key)) {
            cout << "Coordinates out of bounds during ray casting" << endl;
            return false;
        }

        ColorOcTreeNode* starting_node = this->search(current_key, depth);

        if (starting_node) {
            if (this->isNodeOccupied(starting_node)) {
                end = this->keyToCoord(current_key, depth);
                return true;
            }
        } 
        else if (!ignore_unknown) {
            end = this->keyToCoord(current_key, depth);
            return false;
        }

        point3d normalized_direction = direction.normalized();

        bool max_range_set = (max_range > 0.0);

        int step[3];
        double t_max[3];
        double t_delta[3];

        for (unsigned int i = 0; i < 3; ++i) {
            if (normalized_direction(i) > 0.0) {
                step[i] = 1;
            }
            else if (normalized_direction(i) < 0.0) {
                step[i] = -1;
            }
            else {
                step[i] = 0;
            }

            if (step[i] != 0) {
                double voxel_border = this->keyToCoord(current_key[i], depth);
                voxel_border += static_cast<double>(step[i] * resolution * 0.5);

                t_max[i] = (voxel_border - origin(i)) / normalized_direction(i);
                t_delta[i] = resolution / fabs(normalized_direction(i));
            }
            else {
                t_max[i] = numeric_limits<double>::max();
                t_delta[i] = numeric_limits<double>::max();
            }
        }

        if (step[0] == 0 && step[1] == 0 && step[2] == 0) {
            cout << "Raycasting in direction (0,0,0) is not possible!" << endl;
            return false;
        }

        double max_range_sq = max_range * max_range;

        // Incremental phase  ---------------------------------------------------------

        bool done = false;

        while (!done) {
            unsigned int dim;

            if (t_max[0] < t_max[1]) {
                dim = (t_max[0] < t_max[2]) ? 0 : 2;
            }
            else {
                dim = (t_max[1] < t_max[2]) ? 1 : 2;
            }

            if ((step[dim] < 0 && current_key[dim] == 0) ||
                (step[dim] > 0 && current_key[dim] == 2 * this->tree_max_val - 1)) {

                cout << "Coordinate hit bounds in dim" << dim << ", aborting raycast" << endl;
                end = this->keyToCoord(current_key, depth);
                return false; 
            }

            current_key[dim] += step[dim];
            t_max[dim] += t_delta[dim];

            end = this->keyToCoord(current_key, depth);

            if (max_range_set) {
                double distance_from_origin_sq = 0.0;

                for (unsigned int j = 0; j < 3; ++j) { 
                    distance_from_origin_sq += ((end(j) - origin(j)) * (end(j) * origin(j)));
                }

                if (distance_from_origin_sq > max_range_sq) {
                    return false;
                }
            }

            ColorOcTreeNode* current_node = this->search(current_key, depth);

            if (current_node) {
                if (this->isNodeOccupied(current_node)) {
                    done = true;
                    break;
                }
                else if (!ignore_unknown) {
                    return false;
                }
            }
            else {
                if (evaluateRayCast(target, end, depth)) {
                    done = true;
                    break;
                }
            }
        }

        return true;
    }


    bool CustomOcTree::evaluateRayCast(const point3d& target_point, 
        const point3d& end_point, int depth) const {

        OcTreeKey target_key, end_key;

        if (this->coordToKeyChecked(target_point, depth, target_key) &&
            this->coordToKeyChecked(end_point, depth, end_key)) {

            return (target_key == end_key);
        }

        return false;
    }
}