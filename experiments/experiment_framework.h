// File: experiment_framework.h
#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <fstream>      // For file I/O (CSV)
#include <iomanip>      // For setprecision
#include <chrono>       // For timing
#include <numeric>      // For std::accumulate
#include <random>       // For std::mt19937 and seeds
#include <stdexcept>    // For exception handling
#include <map>          // For experiment 4 results
#include <cstdlib>      // For system()
#include <filesystem> 

// --- ADDED: WINDOWS HEADERS FOR MEMORY ---
#include <windows.h>
#include <psapi.h> // For GetProcessMemoryInfo
#pragma comment(lib, "psapi.lib") 

// Include your provided headers by looking up one directory (to ../)
// and then into their respective folders.
#include "../datagen/data_framework.h"
#include "../algo/uav_solver_recursive.h"

// --- 🔬 EXPERIMENT CONFIGURATION ---
const int NUM_ITERATIONS = 300; 
const std::string RESULTS_DIR = "results/";
const std::string RESULTS_PREFIX = "exp_";

// --- (NEW) DEFAULT PARAMETERS FOR HEAVY UAV EXPERIMENTS ---
const double BASE_TIME_STEP = 60.0;
const double BASE_CHARGE_STEP = 35.0;
const int BASE_DECISION_LEVELS = 20;

// --- (NEW) GLOBAL STATE SPACE CONSTANTS ---
const double GLOBAL_MAX_CHARGE = 3970.0; // Wh (From your getHeavyUAVParams())
const double GLOBAL_MAX_TIME_HORIZON = 30000.0; // 8.33 hours


// --- (MODIFIED) HELPER: Result Struct ---
struct RunResult {
    double total_energy_joules;
    double runtime_sec;
    bool success;
    long peak_memory_kb; // <-- ADDED for ablation study
};
// --- HELPER: CSV File Functions ---
// (No changes)
void setup_csv(const std::string& filename_base, const std::string& header) {
    std::filesystem::create_directories(RESULTS_DIR);
    std::string full_path = RESULTS_DIR + filename_base;
    std::ofstream file(full_path, std::ios::trunc);
    if (!file.is_open()) {
        std::cerr << "Error: Failed to create/open CSV file: " << full_path << std::endl;
        return;
    }
    file << header << "\n";
    file.close();
}

void append_to_csv(const std::string& filename_base, const std::vector<std::string>& row_data) {
    std::string full_path = RESULTS_DIR + filename_base;
    std::ofstream file(full_path, std::ios::app);
    if (!file.is_open()) {
        std::cerr << "Error: Failed to append to CSV file: " << full_path << std::endl;
        return;
    }
    for (size_t i = 0; i < row_data.size(); ++i) {
        file << row_data[i] << (i == row_data.size() - 1 ? "" : ",");
    }
    file << "\n";
    file.close();
}

// --- (Corrected) HELPER: Windows Memory Reader ---
long get_current_memory_kb() {
    PROCESS_MEMORY_COUNTERS pmc;
    HANDLE hProcess = GetCurrentProcess();
    if (NULL == hProcess) return -1;

    long memory_kb = -1;
    if (GetProcessMemoryInfo(hProcess, &pmc, sizeof(pmc))) {
        // This is the correct metric for allocated memory (Commit Charge)
        memory_kb = static_cast<long>(pmc.PagefileUsage / 1024);
    }
    CloseHandle(hProcess);
    return memory_kb;
}

// --- (MODIFIED) HELPER: Get Base Config ---
GeneratorConfig get_base_config() {
    GeneratorConfig config;
    config.uav_params = getHeavyUAVParams();
    config.n_tasks = 20; // Base config, main.cpp will override
    config.m_chargers = 5;
    config.area_width = 15000.0;
    config.area_height = 15000.0;
    config.task_distribution = SpatialDistribution::UNIFORM;
    config.num_clusters = 3;
    config.cluster_std_dev = 1000.0;
    config.base_release_time_stagger = 600.0;
    config.time_window_buffer = 7200.0;
    config.exec_time_percentage = 0.01;
    config.charger_placement = ChargerPlacement::RANDOM;
    config.charger_r1_mean = 7938.0;      // Heavy UAV
    config.charger_r1_std_dev = 1984.5; // Heavy UAV
    config.payload_distribution = PayloadDistribution::NORMAL;
    config.payload_param1 = 0.8; // mean
    config.payload_param2 = 0.2; // stddev
    config.wind_grid_rows = 5;
    config.wind_grid_cols = 5;
    config.wind_speed_mean = 8.0;
    config.wind_speed_std_dev = 2.0;
    config.wind_direction_mean = PI;
    config.wind_direction_std_dev = PI / 8.0;
    return config;
}

// --- HELPER: Core Solver Functions ---

// --- (NEW) OVERLOAD: FOR CALCULATED STATE SPACE ---
// This overload accepts the table sizes and passes them to the solver
RunResult run_on_instance(
    ProblemInstance instance,
    double time_step,
    double charge_step,
    int num_charge_levels,
    int max_time_idx,      // <-- The new parameter
    int max_charge_idx     // <-- The new parameter
) {
    try {
        auto start = std::chrono::high_resolution_clock::now();

        // --- Call the FULL 6-param constructor ---
        UAV_Solver_Recursive solver(
            instance,
            time_step,
            charge_step,
            num_charge_levels,
            max_time_idx,      // <-- Pass it to the solver
            max_charge_idx     // <-- Pass it to the solver
        );

        solver.precompute_base();
        solver.precompute_lookups_and_memo(); 
        
        long peak_mem_kb = get_current_memory_kb(); // Poll memory
        double total_cost_joules = solver.run();

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> runtime_sec = end - start;
        bool success = (total_cost_joules < INF);
        return { total_cost_joules, runtime_sec.count(), success, peak_mem_kb };

    } catch (const std::exception& e) {
        std::cerr << "\n         Solver failed: " << e.what() << std::endl;
        return { INF, 0.0, false, -1 };
    }
}

// --- (!!! NEW FUNCTION ADDED !!!) ---
// This helper function was in your old exp1a.cpp but missing here.
// It bridges the gap between the config and the solver.
RunResult run_single_iteration(const GeneratorConfig& config, unsigned int seed) {
    try {
        // 1. Generate the instance
        ProblemInstance instance = DataGenerator::generateInstance(config, seed);

        // 2. Define the discretization parameters (using the base values)
        double time_step = BASE_TIME_STEP;
        double charge_step = BASE_CHARGE_STEP;
        int num_charge_levels = BASE_DECISION_LEVELS;

        // 3. Calculate the state space dimensions
        // We add a small buffer (+2) for safety.
        int max_time_idx = static_cast<int>(std::round(GLOBAL_MAX_TIME_HORIZON / time_step)) + 2;
        int max_charge_idx = static_cast<int>(std::round(GLOBAL_MAX_CHARGE / charge_step)) + 2;

        // 4. Call the detailed run function
        return run_on_instance(
            instance,
            time_step,
            charge_step,
            num_charge_levels,
            max_time_idx,
            max_charge_idx
        );

    } catch (const std::exception& e) {
        // Catch data generation failures (e.g., "Failed to place task")
        std::cerr << "\n      DataGen failed for seed " << seed << ": " << e.what() << std::endl;
        return { INF, 0.0, false, -1 };
    }
}