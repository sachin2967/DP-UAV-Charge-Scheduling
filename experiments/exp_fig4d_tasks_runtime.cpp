#include "experiment_framework.h"
#include <cmath> // For std::round

// This experiment implements a robust hybrid method:
// 1. It finds N *different* feasible instances (solves instance noise).
// 2. For EACH feasible instance, it runs it K times to get a "stable average" (solves system noise).
// 3. It averages these N "stable averages" to get the final avg_runtime.

int main() {
    std::cout << "Running Experiment 1a (Runtime): n_tasks vs. m_chargers (Hybrid)" << std::endl;

    // --- 4-Line Plot Setup ---
    std::vector<int> m_chargers_range = {2, 5, 12,24};
    std::vector<int> n_tasks_range = {5, 10, 15, 20, 25, 30, 35, 40};
    
    
    std::string filename = RESULTS_PREFIX + "1a_tasks_vs_chargers_runtime_hybrid.csv";
    setup_csv(filename, "m_chargers,n_tasks,avg_runtime_sec,success_rate,num_stable_samples,total_runs");
    
    std::mt19937 seed_rng(std::chrono::system_clock::now().time_since_epoch().count());
    
    // --- HYBRID METHOD PARAMETERS ---
    
    // 1. Target # of *different feasible instances* to sample.
    const int N_TARGET_SAMPLES = 1000; 
    
    // 2. # of times to run *each* feasible instance to average system noise.
    const int K_RUNS_PER_INSTANCE = 1; // 1 is fine if deterministic, higher for noisy systems

    // 3. A safety valve to prevent loops if success rate is too low.
    const int MAX_TOTAL_ATTEMPTS = 1000; 

    // --- SOLVER CONFIGURATION (From Header Globals) ---
    // We pre-calculate these based on the constants in experiment_framework.h
    double time_step = BASE_TIME_STEP;
    double charge_step = BASE_CHARGE_STEP;
    int num_charge_levels = BASE_DECISION_LEVELS;
    
    // Calculate state space dimensions (+2 buffer)
    int max_time_idx = static_cast<int>(std::round(GLOBAL_MAX_TIME_HORIZON / time_step)) + 2;
    int max_charge_idx = static_cast<int>(std::round(GLOBAL_MAX_CHARGE / charge_step)) + 2;

    // --- OUTER LOOP: For the 4 lines ---
    for (int m_chargers : m_chargers_range) {
        std::cout << "--- Testing m_chargers = " << m_chargers << " ---" << std::endl;

        // --- INNER LOOP: For the X-axis ---
        for (int n_tasks : n_tasks_range) {
            std::cout << "    Testing n_tasks = " << n_tasks << "..." << std::flush;
            
            GeneratorConfig config = get_base_config();
            config.n_tasks = n_tasks;
            config.m_chargers = m_chargers;
            
            // This will store the "stable average" from each of the N samples
            std::vector<double> stable_averages;
            int total_runs = 0; // Total # of instances *generated*

            // --- N_TARGET_SAMPLES Loop ---
            // Keep looping until we have N stable averages, or we hit the safety limit
            while (stable_averages.size() < N_TARGET_SAMPLES && total_runs < MAX_TOTAL_ATTEMPTS) {
                
                // 1. Generate a new candidate instance
                total_runs++;
                ProblemInstance instance;
                RunResult first_run_result;
                try {
                    instance = DataGenerator::generateInstance(config, seed_rng());
                    
                    // FIXED: Passed the 6 required arguments to run_on_instance
                    first_run_result = run_on_instance(
                        instance, 
                        time_step, 
                        charge_step, 
                        num_charge_levels, 
                        max_time_idx, 
                        max_charge_idx
                    );

                } catch (const std::exception& e) {
                    continue; // Generation or first run failed, try next
                }

                // 2. Check if this instance is feasible
                if (first_run_result.success) {
                    // --- K_RUNS_PER_INSTANCE Loop ---
                    // It's feasible! Now run it K-1 more times
                    std::vector<double> single_instance_runtimes;
                    single_instance_runtimes.push_back(first_run_result.runtime_sec);

                    for (int k = 1; k < K_RUNS_PER_INSTANCE; ++k) {
                        // FIXED: Passed the 6 required arguments here as well
                        RunResult result = run_on_instance(
                            instance, 
                            time_step, 
                            charge_step, 
                            num_charge_levels, 
                            max_time_idx, 
                            max_charge_idx
                        );
                        
                        if (result.success) { 
                            single_instance_runtimes.push_back(result.runtime_sec);
                        }
                    }
                    
                    // 3. Calculate the "stable average" for this one instance
                    if (!single_instance_runtimes.empty()) {
                        double stable_avg_for_this_instance = std::accumulate(
                            single_instance_runtimes.begin(), 
                            single_instance_runtimes.end(), 0.0) / single_instance_runtimes.size();
                        
                        // 4. Add it to our list of stable averages
                        stable_averages.push_back(stable_avg_for_this_instance);
                    }
                }
                // If first_run_result.success was false, we just loop and try a new instance
            }
            // --- End of N_TARGET_SAMPLES loop ---
            
            // --- Final Calculations ---
            
            // This is the average of the "stable averages"
            double final_avg_runtime = (stable_averages.size() > 0) ?
                (std::accumulate(stable_averages.begin(), stable_averages.end(), 0.0) / stable_averages.size())
                : 0.0;
            
            // Success rate is (number of successes) / (total attempts)
            double success_rate = (total_runs > 0) ?
                (static_cast<double>(stable_averages.size()) / total_runs)
                : 0.0;
            
            append_to_csv(filename, {
                std::to_string(m_chargers),
                std::to_string(n_tasks), 
                std::to_string(final_avg_runtime),
                std::to_string(success_rate),
                std::to_string(stable_averages.size()), // How many samples we actually got
                std::to_string(total_runs)
            });
            std::cout << " Done. (Avg Runtime: " << final_avg_runtime << "s, SR: " 
                      << (success_rate * 100) << "%, " << stable_averages.size() << "/" << total_runs << " attempts)" << std::endl;
        }
    }
    
    std::cout << "Experiment 1a (Runtime) complete." << std::endl;
    return 0;
}