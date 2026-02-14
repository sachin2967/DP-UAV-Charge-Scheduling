// This file runs the untime & Quality Experiments"
#include "experiment_framework.h"
//  Tests Charge Step (X-axis) vs. Decision Levels (Lines)

int main() {
    
    // --- 1. Setup Base Problem Config ---
    std::cout << "--- 🔬 Running Monte Carlo Interaction Study ---" << std::endl;
    std::cout << "Averaging " << NUM_ITERATIONS << " random instances for EACH parameter value." << std::endl;
    std::cout << "Global Time Horizon: " << GLOBAL_MAX_TIME_HORIZON << "s" << std::endl;
    std::cout << "Global Max Charge: " << GLOBAL_MAX_CHARGE << "Wh" << std::endl;

    GeneratorConfig base_config = get_base_config();
    base_config.n_tasks = 20; 
    base_config.m_chargers = 5;
    
    std::mt19937 seed_rng(std::chrono::system_clock::now().time_since_epoch().count());

    // --- 1.5. Generate a fixed set of instances (THE FIX) ---
    // This implements "Common Random Numbers" (CRN).
    // All parameter combinations will be tested on the *exact same* set of problems,
    // which removes random noise and reveals the true trends.
    std::cout << "Generating " << NUM_ITERATIONS << " fixed instances for all experiments..." << std::endl;
    std::vector<ProblemInstance> fixed_instances;
    fixed_instances.reserve(NUM_ITERATIONS);
    for (int i = 0; i < NUM_ITERATIONS; ++i) {
       fixed_instances.push_back(DataGenerator::generateInstance(base_config, seed_rng()));
    }
    std::cout << "  ...Instance generation complete." << std::endl;


    // --- Define the ranges for the interaction (VERIFIED) ---
    std::vector<double> charge_step_range = {5.0, 15.0, 25.0, 35.0, 50.0, 75.0};
    std::vector<int> decision_levels_range = {4,8,12,20,30};

    //  Experiment: Charge Step (X) vs. Decision Levels (Lines) ---
    std::cout << "\nRunning Ablation B: Charge Step (X) vs. Decision Levels (Lines)..." << std::endl;
    std::string exp_b_filename = RESULTS_PREFIX + "mc_interact_charge_vs_decision.csv";
    setup_csv(exp_b_filename, "charge_step_wh,decision_levels,avg_peak_memory_kb,avg_energy_joules,avg_runtime_sec,success_rate");

    // Calculate the baseline time table size (constant for this exp)
    int const_time_idx = static_cast<int>(round(GLOBAL_MAX_TIME_HORIZON / BASE_TIME_STEP)) + 1;

    for (double charge_step : charge_step_range) { // X-Axis
        // Calculate the charge table size (changes with X-axis)
        int calculated_charge_idx = static_cast<int>(round(GLOBAL_MAX_CHARGE / charge_step)) + 1;

        for (int decision_levels : decision_levels_range) { // Lines
            std::cout << "  Testing charge_step=" << charge_step << ", decisions=" << decision_levels << "..." << std::flush;
            std::vector<double> energies, runtimes, memories;
            int success_count = 0;

            for (int i = 0; i < NUM_ITERATIONS; ++i) {
                try {
                    // **THE FIX**: Use the same instance from the pre-generated vector
                    const ProblemInstance& instance = fixed_instances[i]; 
                    RunResult r = run_on_instance(
                        instance,
                        BASE_TIME_STEP,    
                        charge_step,       
                        decision_levels,    
                        const_time_idx,    
                        calculated_charge_idx 
                    );
                    if (r.success) {
                        energies.push_back(r.total_energy_joules);
                        runtimes.push_back(r.runtime_sec);
                        memories.push_back(static_cast<double>(r.peak_memory_kb));
                        success_count++;
                    } else if (r.peak_memory_kb > 0) {
                        memories.push_back(static_cast<double>(r.peak_memory_kb));
                    }
                } catch (const std::exception& e) {}
            }
            double avg_energy = (success_count > 0) ? (std::accumulate(energies.begin(), energies.end(), 0.0) / success_count) : INF;
            double avg_runtime = (success_count > 0) ? (std::accumulate(runtimes.begin(), runtimes.end(), 0.0) / success_count) : 0.0;
            double avg_memory = (memories.size() > 0) ? (std::accumulate(memories.begin(), memories.end(), 0.0) / memories.size()) : 0.0;
            double success_rate = static_cast<double>(success_count) / NUM_ITERATIONS;

            append_to_csv(exp_b_filename, {
                std::to_string(charge_step),
                std::to_string(decision_levels),
                std::to_string(avg_memory),
                std::to_string(avg_energy),
                std::to_string(avg_runtime),
                std::to_string(success_rate)
            });
            std::cout << " Done. (Avg Mem: " << static_cast<long>(avg_memory) << " KB, SR: " << success_rate*100 << "%)" << std::endl;
        }
    }
    std::cout << "Ablation complete." << std::endl;
    return 0;
}