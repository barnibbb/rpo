#pragma once
#include <string>
#include <unordered_map>
#include <fstream>

using namespace std;

namespace rpo
{
    using Params = unordered_map<string, string>;

    enum class Crossover  { Uniform, TwoPoint };
    enum class Mutation   { Normal, Uniform };
    enum class Selection  { FitnessProportionate, Random, Tournament, Truncation };
    enum class RayTracing { None, TwoDimensional, ThreeDimensional };

    const Crossover CROSSOVERS[2] { 
        Crossover::Uniform, 
        Crossover::TwoPoint
    };
    const Mutation MUTATIONS[2] { 
        Mutation::Normal,
        Mutation::Uniform
    };
    const Selection SELECTIONS[4] { 
        Selection::FitnessProportionate,
        Selection::Random,
        Selection::Tournament,
        Selection::Truncation
    };
    const RayTracing RAYTRACINGS[3] { 
        RayTracing::None,
        RayTracing::TwoDimensional,
        RayTracing::ThreeDimensional
    };


    struct Parameters
    {
        void setValues(const string& parameters_file);

        // Paths -------------------------------------------------------------------
        string model_path;
        string coarse_lamp_model;
        string fine_lamp_model;
        string precomputation_folder;

        // 3D model parameters -----------------------------------------------------
        int depth = 15;
        
        double resolution = 0.1;
        double reachable_region_x = 0;
        double reachable_region_y = 0;
        double ground_level;

        bool smooth_model = false;

        // Dose calculation parameters ---------------------------------------------
        double lamp_height = 1.2;
        double lamp_offset = 0.5;
        double lamp_range = 20;
        double lamp_power = 80;

        bool final_calculation = false;
        bool precompute_irradiance = true;

        int precomputation_grid_depth = 13;

        RayTracing ray_traing = RayTracing::ThreeDimensional;

        // Plan generation parameters ----------------------------------------------
        int number_of_positions = 1;
        int plan_element_size = 3;
        int individual_size = 3;
        int max_generations = 10;
        int population_size = 20;

        double exposure_limit = 100;
        double coverage = 0.9;
        double overall_time = 1800;
        double radiation_base_time = 1800;

        int number_of_crossovers = 20;
        int number_of_mutations = 10;

        double gene_mutation_probability = 0.7;
        double coordinate_mutation_parameter = 0.2;
        double time_mutation_parameter = 0.3;

        Crossover crossover = Crossover::Uniform;
        Mutation mutation = Mutation::Uniform;
        Selection crossover_selection = Selection::FitnessProportionate;
        Selection mutation_selection = Selection::FitnessProportionate;
        Selection survival_selection = Selection::Truncation;

        double min_distance_from_obstacles = 0.3;

        // Extra parameters --------------------------------------------------------
        bool delete_ceiling = false;
        bool expand_model = false;
        bool modify_normals = true;
    };
}
