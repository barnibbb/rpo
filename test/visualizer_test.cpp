#include "ros_visualizer.h"

int main (int argc, char** argv)
{
    const string parameters_file = "/home/barni/rpo_ws/src/rpo/files/hospital_parameters.txt";

    rpo::Parameters parameters;

    parameters.setValues(parameters_file);

    const string file = "/home/barni/rpo_ws/src/rpo/models/hospital_model.ot";

    ifstream f (file);

    unique_ptr<octomap::ColorOcTree> color_octree = nullptr;

    color_octree.reset(dynamic_cast<octomap::ColorOcTree*>(octomap::AbstractOcTree::read(f)));

    unique_ptr<rpo::AugmentedOcTree> augmented_model = make_unique<rpo::AugmentedOcTree>(color_octree->getResolution());


    augmented_model->setParameters(parameters);

    augmented_model->computeGroundLevel();
    augmented_model->computeGroundZone();
    augmented_model->computeReachableElements();
    augmented_model->computeNormals();
    augmented_model->computePlanes();
    augmented_model->computeGridElements();

    rpo::DoseCalculator dose_calculator(*augmented_model);

    return 0;
}
