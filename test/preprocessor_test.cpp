#include <iostream>
#include <string>
#include <memory>

#include "augmented_model.h"

using namespace std;

int main (int argc, char** argv)
{   
    const string parameters_file = "/home/barni/rpo_ws/src/rpo/files/hospital_parameters.txt";

    rpo::Parameters parameters;
    parameters.setValues(parameters_file);

    rpo::AugmentedModel augmented_model(parameters);

    try
    {
        augmented_model.computeGroundLevel();
        augmented_model.computeGroundZone();
        augmented_model.computeReachableElements();
        augmented_model.computeNormals();
        augmented_model.computePlanes();
        augmented_model.computeGridElements();

        augmented_model.logResults();
    }
    catch(const exception& error)
    {
        cerr << error.what() << endl;
    }

    return 1;
}
