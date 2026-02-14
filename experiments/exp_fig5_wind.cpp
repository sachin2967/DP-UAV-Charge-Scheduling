#include "experiment_framework.h"

int main() {
    std::cout << "Running Experiment: Wind Speed Mean vs. Area Size" << std::endl;
    
    // 1. Define the parameters you want to sweep
    
    // Outer loop for "multiple plots"
    std::vector<double> area_sizes_m = {
        5000.0,  // 5km x 5km
        10000.0, // 10km x 10km
        15000.0, // 15km x 15km
        20000.0  // 20km x 20km
    };
    
    // Inner loop for the "x-axis"
    // --- MODIFIED: Your new wind speed range ---
    std::vector<double> wind_speed_mean_range = {
        2.0,
        4.0,
        6.0,
        8.0,  // Base value
        10.0,
        12.0
    };

    // 2. Setup ONE CSV file with columns for all parameters
    std::string filename = RESULTS_PREFIX + "wind_mean_vs_area.csv";
    setup_csv(filename, "area_size_m,wind_speed_mean,avg_energy_joules,success_rate");
    
    std::mt19937 seed_rng(std::chrono::system_clock::now().time_since_epoch().count());

    // 3. OUTER LOOP: Iterate over area sizes
    for (double area_size : area_sizes_m) {
        std::cout << "--- Testing Area Size = " << area_size << "m ---" << std::endl;

        // 4. INNER LOOP: Iterate over wind speed mean
        for (double wind_mean : wind_speed_mean_range) {
            std::cout << "      Testing Wind Speed Mean = " << wind_mean << " m/s..." << std::flush;
            
            GeneratorConfig config = get_base_config();
            
            // --- Set the loop parameters ---
            config.area_width = area_size;
            config.area_height = area_size;
            
            // Set the wind parameters for this specific run
            config.wind_speed_mean = wind_mean; // <-- Set variable wind MEAN
            config.wind_speed_std_dev = 2.0;    // <-- Keep base wind STDDEV
            
            std::vector<double> energies;
            int success_count = 0;

            for (int i = 0; i < NUM_ITERATIONS; ++i) {
                RunResult result = run_single_iteration(config, seed_rng());
                
                if (result.success) {
                    energies.push_back(result.total_energy_joules);
                    success_count++;
                }
            }
            
            double avg_energy = (success_count > 0) ? (std::accumulate(energies.begin(), energies.end(), 0.0) / success_count) : INF;
            double success_rate = static_cast<double>(success_count) / NUM_ITERATIONS;
            
            // 5. Append to CSV, including all parameter values
            append_to_csv(filename, {
                std::to_string(area_size),
                std::to_string(wind_mean),
                std::to_string(avg_energy),
                std::to_string(success_rate)
            });
            std::cout << " Done. (SR: " << (success_rate * 100) << "%)" << std::endl;
        }
    }
    
    std::cout << "Experiment (Wind Mean vs. Area) complete." << std::endl;
    return 0;
}