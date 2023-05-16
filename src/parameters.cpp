#include "parameters.h"

using namespace std;

namespace rpo
{
    void Parameters::setValues(const string& parameters_file)
    {
        ifstream file(parameters_file);

        Params params;

        if (file.is_open())
        {
            string line;

            while (getline(file, line))
            {
                if (size_t pos = line.find(':'); pos != string::npos)
                {
                    const string param_name = line.substr(0, pos);
                    const string param_value = line.substr(pos + 2, line.length() - pos - 2);

                    params[param_name] = param_value;
                }
            }
            
            file.close();
        }
        else
        {
            throw runtime_error { "Could not read parameters file" };
        }


        // Paths -------------------------------------------------------------------
        model_path = params["model path"];
        coarse_lamp_model = params["coarse lamp model"];
        fine_lamp_model = params["fine lamp model"];
        precomputation_folder = params["precomputation folder"];

        // 3D model parameters -----------------------------------------------------
        depth = stoi(params["depth"]);
        
        resolution = stod(params["resolution"]);
        reachable_region_x = stod(params["reachable region x"]);
        reachable_region_y = stod(params["reachable region y"]);

        smooth_model = params["smooth model"] == "true";

        // Dose calculation parameters ---------------------------------------------
        lamp_height = stod(params["lamp height"]);
        lamp_offset = stod(params["lamp offset"]);
        lamp_range = stod(params["lamp range"]);
        lamp_power = stod(params["lamp power"]);

        final_calculation = params["final calculation"] == "true";
        precompute_irradiance = params["precompute irradiance"] == "true";

        precomputation_grid_depth = stoi(params["precomputation grid depth"]);

        ray_traing = RAYTRACINGS[stoi(params["ray tracing"])];

        // Plan generation parameters ----------------------------------------------
        number_of_positions = stoi(params["number of positions"]);
        plan_element_size = stoi(params["plan element size"]);

        individual_size = number_of_positions * plan_element_size;

        max_generations = stoi(params["max generations"]);
        population_size = stoi(params["population size"]);

        exposure_limit = stod(params["exposure limit"]);
        coverage = stod(params["coverage"]);
        overall_time = stod(params["overall time"]);

        radiation_base_time = overall_time / static_cast<double>(number_of_positions);

        number_of_crossovers = stoi(params["number of crossovers"]);
        number_of_mutations = stoi(params["number of mutations"]);

        gene_mutation_probability = stod(params["gene mutation probability"]);
        coordinate_mutation_parameter = stod(params["coordinate mutation parameter"]);
        time_mutation_parameter = stod(params["time mutation parameter"]);

        crossover = CROSSOVERS[stoi(params["crossover"])];
        mutation = MUTATIONS[stoi(params["mutation"])];
        crossover_selection = SELECTIONS[stoi(params["crossover selection"])];
        mutation_selection = SELECTIONS[stoi(params["mutation selection"])];
        survival_selection = SELECTIONS[stoi(params["survival selection"])];

        min_distance_from_obstacles = stod(params["min distance from obstacles"]);

        // Extra parameters --------------------------------------------------------
        delete_ceiling = params["delete ceiling"] == "true";
        expand_model = params["expand model"] == "true";
        modify_normals = params["modify normals"] == "true";
    }
}
