#include <fstream>
#include <iostream>

#include "parameters.h"

using namespace std;

int main(int argc, char** argv)
{
    const string parameters_file = "/home/barni/rpo_ws/src/rpo/files/hospital_parameters.txt";

    try
    {
        rpo::Parameters parameters;
        parameters.setValues(parameters_file);

        cout << "model path: " << parameters.model_path << '\n'
             << "coarse lamp model: " << parameters.coarse_lamp_model << '\n'
             << "fine lamp model: " << parameters.fine_lamp_model << '\n'
             << "precomputation folder: " << parameters.precomputation_folder << '\n'
             << "depth: " << parameters.depth << '\n'
             << "resolution: " << parameters.resolution << '\n'
             << "reachable region x: " << parameters.reachable_region_x << '\n'
             << "reachable region x: " << parameters.reachable_region_y << '\n'
             << "smooth model: " << parameters.smooth_model << '\n'
             << "lamp height: " << parameters.lamp_height << '\n'
             << "lamp offset: " << parameters.lamp_offset << '\n'
             << "lamp range: " << parameters.lamp_range << '\n'
             << "lamp power: " << parameters.lamp_power << '\n'
             << "precompute irradiance: " << parameters.precompute_irradiance << '\n'
             << "precomputation grid depth: " << parameters.precomputation_grid_depth << '\n'
             << "ray tracing: " << static_cast<underlying_type<rpo::RayTracing>::type>(parameters.ray_traing) << '\n'
             << "number of positions: " << parameters.number_of_positions << '\n'
             << "plan element size: " << parameters.plan_element_size << '\n'
             << "individual size: " << parameters.individual_size << '\n'
             << "max generations: " << parameters.max_generations << '\n'
             << "population size: " << parameters.population_size << '\n'
             << "exposure limit: " << parameters.exposure_limit << '\n'
             << "coverage: " << parameters.coverage << '\n'
             << "overall time: " << parameters.overall_time << '\n'
             << "raidation base time: " << parameters.radiation_base_time << '\n'
             << "number of crossovers: " << parameters.number_of_crossovers << '\n'
             << "number of mutations: " << parameters.number_of_mutations << '\n'
             << "gene mutation probability: " << parameters.gene_mutation_probability << '\n'
             << "coordinate mutation parameter: " << parameters.coordinate_mutation_parameter << '\n'
             << "time mutation parameter: " << parameters.time_mutation_parameter << '\n'
             << "crossover: " << static_cast<underlying_type<rpo::Crossover>::type>(parameters.crossover) << '\n'
             << "mutation: " << static_cast<underlying_type<rpo::Mutation>::type>(parameters.mutation) << '\n'
             << "crossover selection: " << static_cast<underlying_type<rpo::Selection>::type>(parameters.crossover_selection) << '\n'
             << "mutation selection: " << static_cast<underlying_type<rpo::Selection>::type>(parameters.mutation_selection) << '\n'
             << "survival selection: " << static_cast<underlying_type<rpo::Selection>::type>(parameters.survival_selection) << '\n'
             << "min distance from obstacles: " << parameters.min_distance_from_obstacles << '\n'
             << "delete ceiling: " << parameters.delete_ceiling << '\n'
             << "expand model: " << parameters.expand_model << '\n'
             << "modify normals: " << parameters.modify_normals << '\n';
    }
    catch(const exception& error)
    {
        cerr << error.what() << endl;
    }
    

        

    return 0;
}