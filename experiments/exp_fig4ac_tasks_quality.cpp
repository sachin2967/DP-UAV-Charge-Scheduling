#include "experiment_framework.h"

int main() {
    std::cout << "Running Experiment 1a (Energy): n_tasks vs. m_chargers" << std::endl;
    
    // 1. Define the parameter you want to sweep
    std::vector<int> m_chargers_range = {2,5, 12,24}; // <-- YOUR 4 LINES
    std::vector<int> n_tasks_range = {5, 10, 15, 20, 25, 30, 35, 40};

    // 2. Setup ONE CSV file with a new column for the parameter
    std::string filename = RESULTS_PREFIX + "1a_tasks_vs_chargers.csv";
    // Add the new parameter to the header
    setup_csv(filename, "m_chargers,n_tasks,avg_energy_joules,success_rate");
    
    std::mt19937 seed_rng(std::chrono::system_clock::now().time_since_epoch().count());

    // 3. OUTER LOOP: Iterate over the new parameter
    for (int m_chargers : m_chargers_range) {
        std::cout << "--- Testing m_chargers = " << m_chargers << " ---" << std::endl;

        // 4. INNER LOOP: Your existing experiment logic
        for (int n_tasks : n_tasks_range) {
            std::cout << "    Testing n_tasks = " << n_tasks << "..." << std::flush;
            
            GeneratorConfig config = get_base_config();
            config.n_tasks = n_tasks;
            config.m_chargers = m_chargers; // <-- Set the outer param
            
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
            
            // 5. Append to CSV, including the new parameter value
            append_to_csv(filename, {
                std::to_string(m_chargers), // <-- Add the new value
                std::to_string(n_tasks), 
                std::to_string(avg_energy), 
                std::to_string(success_rate)
            });
            std::cout << " Done. (SR: " << (success_rate * 100) << "%)" << std::endl;
        }
    }
    
    std::cout << "Experiment 1a (vs. chargers) complete." << std::endl;
    return 0;
}