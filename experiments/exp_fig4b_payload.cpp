#include "experiment_framework.h"

int main() {
    std::cout << "Running Experiment: Mean Payload vs. Number of Chargers" << std::endl;
    
    // 1. Define the parameters you want to sweep
    
    // Outer loop for "multiple plots"
    // We test a range from scarce to abundant chargers.
    std::vector<int> m_chargers_range = {
       12
    };
    
    // Inner loop for the "x-axis"
    // We vary the mean payload around your base of 0.8
    // Note: A mean of 1.2 will be heavily clipped by the 1.4 cap.
    std::vector<double> payload_mean_range = { 
        0.2,
        0.4,
        0.6,
        0.8,  
        1.0,
        1.2   // Very heavy (approaching 1.4 cap)
    };

    // 2. Setup ONE CSV file with columns for all parameters
    std::string filename = RESULTS_PREFIX + "payload_mean_vs_chargers.csv";
    setup_csv(filename, "m_chargers,payload_mean,avg_energy_joules,success_rate");
    
    std::mt19937 seed_rng(std::chrono::system_clock::now().time_since_epoch().count());

    // 3. OUTER LOOP: Iterate over number of chargers
    for (int num_chargers : m_chargers_range) {
        std::cout << "--- Testing m_chargers = " << num_chargers << " ---" << std::endl;

        // 4. INNER LOOP: Iterate over payload mean
        for (double payload_mean : payload_mean_range) {
            std::cout << "      Testing Payload Mean = " << payload_mean << " kg..." << std::flush;
            
            GeneratorConfig config = get_base_config();
            
            // --- Set the loop parameters ---
            config.m_chargers = num_chargers;  // Set outer loop var
            config.payload_param1 = payload_mean; // Set inner loop var
            
            // Keep other parameters (like stddev) at their base values
            config.payload_distribution = PayloadDistribution::NORMAL;
            config.payload_param2 = 0.2; // Keep base stddev constant
            
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
                std::to_string(num_chargers),
                std::to_string(payload_mean),
                std::to_string(avg_energy),
                std::to_string(success_rate)
            });
            std::cout << " Done. (SR: " << (success_rate * 100) << "%)" << std::endl;
        }
    }
    
    std::cout << "Experiment (Payload Mean vs. Chargers) complete." << std::endl;
    return 0;
}