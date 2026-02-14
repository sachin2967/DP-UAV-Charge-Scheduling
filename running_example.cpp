// --- Main executable for a single test run ---
// This file will:
// 1. Configure a specific test case (HEAVY UAV) designed to force multiple charger uses.
// 2. Call the "Feasibility-First" Data Generator.
// 3. Print a detailed summary of the generated problem.
// 4. Call the Recursive Solver.
// 5. Print the step-by-step solution path.

// We assume headers are in the parent directory in 'datagen' and 'algo' folders
#include "../datagen/data_framework.h"     // Assumes this is your CORRECTED version
#include "../algo/uav_solver_recursive.h" // Assumes this is your CORRECTED version

#include <iostream>
#include <iomanip> // Needed for setw, left
#include <chrono>  // For timing the solver
#include <stdexcept> // For std::exception

// Using directives for cleaner code
using std::cout;
using std::endl;
using std::string;
using std::to_string;
using std::left;
using std::setw;
using std::fixed;
using std::setprecision;
using std::cerr;

// NOTE: getLightUAVParams() and getHeavyUAVParams() are in data_framework.h

// --- Helper Function to Print Instance Details ---
// (This function is unchanged as it's generic)
void print_problem_details(const ProblemInstance& prob, unsigned int seed) {
    cout << "\n--- 📜 Problem Instance Details (Seed " << seed << ") ---" << endl;
    cout << fixed << setprecision(2);

    // UAV Params
    cout << "\nUAV Parameters:" << endl;
    cout << "  " << left << setw(15) << "Base Mass:"      << prob.uav_params.base_mass << " kg" << endl;
    cout << "  " << left << setw(15) << "Max Battery:"    << prob.uav_params.max_battery << " Wh" << endl;
    cout << "  " << left << setw(15) << "Groundspeed:"    << prob.uav_params.v_groundspeed << " m/s" << endl;
    cout << "  " << left << setw(15) << "P_exec:"         << prob.uav_params.p_ex << " kW" << endl;
    cout << "  " << left << setw(15) << "P_wait:"         << prob.uav_params.p_wt << " kW" << endl;
    cout << "  " << left << setw(15) << "k1, k2, k3:"     << prob.uav_params.k1 << ", " << prob.uav_params.k2 << ", " << prob.uav_params.k3 << endl;

    // Locations
    cout << "\nLocations:" << endl;
    cout << "  " << left << setw(5) << "ID" << " | " << setw(13) << "Type" << " | " << setw(10) << "X" << " | " << "Y" << endl;
    cout << "  " << string(45, '-') << endl;
    cout << "  " << left << setw(5) << "0" << " | " << setw(13) << "Start Depot" << " | " << setw(10) << prob.start_depot.x << " | " << prob.start_depot.y << endl;
    int i = 1;
    for(const auto& task : prob.tasks) {
        cout << "  " << left << setw(5) << i++ << " | " << setw(13) << ("Task " + to_string(task.id)) << " | " << setw(10) << task.loc.x << " | " << task.loc.y << endl;
    }
    cout << "  " << left << setw(5) << i++ << " | " << setw(13) << "End Depot" << " | " << setw(10) << prob.end_depot.x << " | " << prob.end_depot.y << endl;
    for(const auto& charger : prob.chargers) {
        cout << "  " << left << setw(5) << i++ << " | " << setw(13) << ("Charger " + to_string(charger.id)) << " | " << setw(10) << charger.loc.x << " | " << charger.loc.y << endl;
    }

    // Task Details
    cout << "\nTask Details:" << endl;
    cout << "  " << left << setw(5) << "ID" << " | " << setw(12) << "Payload (kg)" << " | " << setw(10) << "Exec (s)" << " | " << setw(13) << "Release (s)" << " | " << "Deadline (s)" << endl;
    cout << "  " << string(70, '-') << endl;
    for(const auto& task : prob.tasks) {
        cout << "  " << left << setw(5) << task.id << " | " << setw(12) << task.payload << " | "
             << setw(10) << task.exec_time << " | " << setw(13) << task.release_time << " | " << task.deadline << endl;
    }

    // Charger Details
    cout << "\nCharger Details:" << endl;
    cout << "  " << left << setw(5) << "ID" << " | " << "R1, R2, R3 (Wh/hr)" << endl;
    cout << "  " << string(40, '-') << endl;
    for(const auto& charger : prob.chargers) {
        cout << "  " << left << setw(5) << charger.id << " | "
             << charger.r1 << ", " << charger.r2 << ", " << charger.r3 << endl;
    }

    // Wind Grid
    cout << "\nWind Grid:" << endl;
    cout << "  Area Dimensions: " << prob.area_width << "m (W) x " << prob.area_height << "m (H)" << endl;
    cout << "  Grid Size:       " << prob.wind_grid_cols << " cols x " << prob.wind_grid_rows << " rows" << endl;
    cout << "  Zone Size:       " << prob.wind_zone_width << "m (W) x " << prob.wind_zone_height << "m (H)" << endl;

    cout << "  " << left << setw(10) << "(Row, Col)" << " | "
         << setw(12) << "Speed (m/s)" << " | "
         << setw(15) << "Direction (rad)" << " | "
         << setw(12) << "Vx (E+/W-)" << " | " << "Vy (N+/S-)" << endl;
    cout << "  " << string(70, '-') << endl;
    for(int r = 0; r < prob.wind_grid_rows; ++r) {
        for(int c = 0; c < prob.wind_grid_cols; ++c) {
            string grid_pos = "(" + to_string(r) + ", " + to_string(c) + ")";
            const Wind& wind = prob.wind_grid[r][c];

            double Vw_x = wind.speed * cos(wind.direction);
            double Vw_y = wind.speed * sin(wind.direction);

            cout << "  " << left << setw(10) << grid_pos << " | "
                 << setw(12) << wind.speed << " | "
                 << setw(15) << wind.direction << " | "
                 << setw(12) << Vw_x << " | " << Vw_y << endl;
        }
    }
    cout << "--------------------------------------------------\n" << endl;
}


// --- 🔬 MAIN TEST RUNNER ---
int main() {
    cout << "--- Running Single Test Case (Heavy UAV) ---" << endl;

    // 1. Create a Generator Configuration
    GeneratorConfig config;

    // --- Configuration tuned for HEAVY UAV (DJI FlyCart 30) ---
    config.n_tasks = 10;
    config.m_chargers = 2;
    config.area_width = 15000.0;
    config.area_height = 15000.0;
    
    config.payload_distribution = PayloadDistribution::NORMAL;
    config.payload_param1 = 0.8;
    config.payload_param2 = 0.2;
    
    config.wind_speed_mean = 8.0;
    config.wind_speed_std_dev = 2.0;
    config.wind_direction_mean = PI;
    config.wind_direction_std_dev = PI / 8.0;

    // *** MODIFICATION ***
    // Make sure your getHeavyUAVParams() in data_framework.h
    config.uav_params = getHeavyUAVParams();
    
    config.wind_grid_rows = 4;
    config.wind_grid_cols = 4;
    // --- End of specific config ---

    config.task_distribution = SpatialDistribution::UNIFORM;
    config.num_clusters = 3;
    config.cluster_std_dev = 3000.0;
    config.charger_placement = ChargerPlacement::GRID;
    
    config.base_release_time_stagger = 600.0;
    config.time_window_buffer = 7200.0;
    config.exec_time_percentage = 0.01; 

    config.charger_r1_mean = 7938.0;
    config.charger_r1_std_dev = 1984.5;

    // ====================================================================
    // --- SECTION 1: MODIFIED ---
    // We no longer set discretization params in the config.
    // They are removed from here.
    // ====================================================================
    // (config.time_step = 60.0;)       <-- DELETED
    // (config.charge_step = 35.0;)    <-- DELETED
    // (config.max_time_idx = 1500;)   <-- DELETED
    // (config.max_charge_idx = ...;)  <-- DELETED


    unsigned int seed = 42;

    try {
        // 2. Generate
        cout << "Generating instance with seed " << seed << "..." << endl;
        cout << "(Note: Using HEAVY UAV with " << config.uav_params.max_battery << "Wh battery)" << endl;
        auto gen_start = std::chrono::high_resolution_clock::now();
        ProblemInstance instance = DataGenerator::generateInstance(config, seed);
        auto gen_end = std::chrono::high_resolution_clock::now();
        cout << "   - Instance generated in " << std::chrono::duration_cast<std::chrono::milliseconds>(gen_end - gen_start).count() << " ms" << endl;

        // ---  PRINT ALL DETAILS (will show heavy UAV stats) ---
        print_problem_details(instance, seed);

        // ====================================================================
        // --- SECTION 2: MODIFIED ---
        // We now define our 3 discretization parameters locally
        // and pass them to the solver's constructor.
        // ====================================================================

        // 3. Create Solver
        double time_step = 60.0;   // State time step (sec)
        double charge_step = 35.0; // State charge step (Wh)
        int num_levels = 20;       // Decision/Action levels

        cout << "Creating solver with:" << endl;
        cout << "  - State Time Step:      " << time_step << " s" << endl;
        cout << "  - State Charge Step:    " << charge_step << " Wh" << endl;
        cout << "  - Decision Charge Levels: " << num_levels << endl;
        
        // Pass all 3 parameters to the corrected constructor
        UAV_Solver_Recursive solver(instance, time_step, charge_step, num_levels);


        // 4. Validate
        cout << "Validating..." << endl;
        ValidationResult result = Validator::checkFeasibility(instance);

        if (result.is_feasible) {
            cout << "Instance is feasible. Starting precomputation and solver..." << endl;

            // 5. Run ALL Precomputation
            auto pre_start = std::chrono::high_resolution_clock::now();
            cout << "Precomputing (Sparse Lookups, Heuristics & Memo)..." << endl;
            solver.precompute_base();
            solver.precompute_lookups_and_memo();
            auto pre_end = std::chrono::high_resolution_clock::now();
            cout << "   - Precomputation finished in " << std::chrono::duration_cast<std::chrono::milliseconds>(pre_end - pre_start).count() << " ms" << endl;


            // 6. Run Solver
            cout << "Running solver..." << endl;

            auto start_time = std::chrono::high_resolution_clock::now();
            double min_energy = solver.run();
            auto end_time = std::chrono::high_resolution_clock::now();

            long long runtime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

            cout << "Solver finished in " << runtime_ms << " ms." << endl;

            // 7. Print Results
            if (min_energy < INF) {
                cout << "SUCCESS: Solution found!" << endl;
                cout << "Total Energy Cost: " << fixed << setprecision(2) << min_energy << " Joules" << endl;
                cout << "Total Energy Cost: " << fixed << setprecision(2) << (min_energy / 3600.0) << " Wh" << endl;

                // 8. Reconstruct Path
                solver.reconstruct_and_print_path();

            } else {
                cout << "FAILURE: Solver could not find a feasible path." << endl;
            }
        } else {
            cout << "FAILURE: Generated Instance Infeasible AFTER generation: " << result.message << endl;
            cout << "         (This suggests an issue in the generator logic or feasibility checks)" << endl;
        }
    } catch (const std::exception& e) {
        cerr << "Critical error: " << e.what() << endl;
        return 1;
    }

    return 0;
}