#include <iostream>

#include "dose_calculator.h"

using namespace std;

int main (int argc, char** argv)
{
    const string parameters_file = "/home/barni/rpo_ws/src/rpo/files/hospital_parameters.txt";

    rpo::Parameters parameters;

    parameters.setValues(parameters_file);

    rpo::AugmentedModel augmented_model(parameters);

    augmented_model.computeGroundLevel();
    augmented_model.computeGroundZone();
    augmented_model.computeReachableElements();
    augmented_model.computeNormals();
    augmented_model.computePlanes();
    augmented_model.computeGridElements();

    rpo::DoseCalculator dose_calculator(augmented_model);

    dose_calculator.computeRayOrigins();

    return 0;
}
