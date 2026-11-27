#pragma once

#include "../datagen/data_framework.h" 
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <string>

using namespace std;

// --- DEFAULT DISCRETIZATION CONSTANTS ---
const double DEFAULT_TIME_STEP = 60.0;
const double DEFAULT_CHARGE_STEP = 35.0;
const int DEFAULT_DECISION_LEVELS = 20;
const int DEFAULT_MAX_TIME_IDX = 1500;    
const int DEFAULT_MAX_CHARGE_IDX = 120;     

// --- 💡 UAV SOLVER STRUCT (RECURSIVE) ---
struct UAV_Solver_Recursive {

    // --- MEMBER VARIABLES ---
    ProblemInstance prob;
    vector<Point> all_locations;
    vector<int> task_to_loc_idx;

    // --- Sparse Lookups ---
    vector<double> direct_leg_time;
    vector<double> direct_leg_energy;
    vector<vector<double>> to_charger_time;
    vector<vector<double>> to_charger_energy;
    vector<vector<double>> from_charger_time;
    vector<vector<double>> from_charger_energy;
    vector<vector<int>> promising_chargers_for_leg;

    // --- Memoization tables ---
    vector<vector<vector<double>>> memo_cost;
    vector<vector<vector<Decision>>> decision_memo;
    
    // --- State Space Discretization (Stored) ---
    double m_time_step;
    double m_charge_step;
    int m_max_time_idx;
    int m_max_charge_idx;

    // --- Action Space ---
    vector<double> admissible_charge_targets_wh;

    // --- CONSTRUCTOR ---
    UAV_Solver_Recursive(
        const ProblemInstance& instance, 
        double time_step = DEFAULT_TIME_STEP,
        double charge_step = DEFAULT_CHARGE_STEP,
        int num_charge_levels = DEFAULT_DECISION_LEVELS, 
        int max_time_idx = DEFAULT_MAX_TIME_IDX,         
        int max_charge_idx = DEFAULT_MAX_CHARGE_IDX      
    ) : prob(instance),
        m_time_step(time_step),
        m_charge_step(charge_step),
        m_max_time_idx(max_time_idx),       
        m_max_charge_idx(max_charge_idx)    
    {
        admissible_charge_targets_wh.clear();
        if (num_charge_levels <= 0) {
            admissible_charge_targets_wh.push_back(prob.uav_params.max_battery);
        } else {
            double step_percent = 1.0 / static_cast<double>(num_charge_levels);
            for (int i = 1; i <= num_charge_levels; ++i) {
                admissible_charge_targets_wh.push_back(prob.uav_params.max_battery * (static_cast<double>(i) * step_percent));
            }
        }
        admissible_charge_targets_wh.push_back(prob.uav_params.max_battery);
        sort(admissible_charge_targets_wh.begin(), admissible_charge_targets_wh.end());
        admissible_charge_targets_wh.erase(
            unique(admissible_charge_targets_wh.begin(), admissible_charge_targets_wh.end()), 
            admissible_charge_targets_wh.end()
        );
    }

    // --- CORE LOGIC METHODS ---

    void precompute_base() {
        all_locations.clear();
        all_locations.push_back(prob.start_depot);
        for (const auto& task : prob.tasks) all_locations.push_back(task.loc);
        all_locations.push_back(prob.end_depot);
        for (const auto& charger : prob.chargers) all_locations.push_back(charger.loc);

        task_to_loc_idx.clear();
        task_to_loc_idx.push_back(0); 
        for (int i = 0; i < prob.n_tasks; ++i) task_to_loc_idx.push_back(i + 1);
        task_to_loc_idx.push_back(prob.n_tasks + 1);
    }

    void precompute_lookups_and_memo() {
        int num_legs = prob.n_tasks + 1; 
        int num_chargers = prob.m_chargers;

        direct_leg_time.assign(num_legs, INF);
        direct_leg_energy.assign(num_legs, INF);
        to_charger_time.assign(num_legs, vector<double>(num_chargers, INF));
        to_charger_energy.assign(num_legs, vector<double>(num_chargers, INF));
        from_charger_time.assign(num_legs, vector<double>(num_chargers, INF));
        from_charger_energy.assign(num_legs, vector<double>(num_chargers, INF));
        promising_chargers_for_leg.assign(num_legs, vector<int>());

        for (int k = 0; k < num_legs; ++k) {
            int from_loc_idx = task_to_loc_idx[k];
            int to_loc_idx = task_to_loc_idx[k + 1];
            double load = prob.payload_suffix_sum[k]; 

            direct_leg_time[k] = PhysicsHelpers::calculate_travel_time(all_locations[from_loc_idx], all_locations[to_loc_idx], prob.uav_params);
            direct_leg_energy[k] = PhysicsHelpers::flight_energy_cost(all_locations[from_loc_idx], all_locations[to_loc_idx], load, prob);

            vector<pair<double, int>> candidates;
            for (int c_id = 0; c_id < num_chargers; ++c_id) {
                int charger_loc_idx = prob.n_tasks + 2 + c_id;
                const auto& charger = prob.chargers[c_id];
                to_charger_time[k][c_id] = PhysicsHelpers::calculate_travel_time(all_locations[from_loc_idx], all_locations[charger_loc_idx], prob.uav_params);
                to_charger_energy[k][c_id] = PhysicsHelpers::flight_energy_cost(all_locations[from_loc_idx], all_locations[charger_loc_idx], load, prob);
                from_charger_time[k][c_id] = PhysicsHelpers::calculate_travel_time(all_locations[charger_loc_idx], all_locations[to_loc_idx], prob.uav_params);
                from_charger_energy[k][c_id] = PhysicsHelpers::flight_energy_cost(all_locations[charger_loc_idx], all_locations[to_loc_idx], load, prob);
            
                double detour_time_sec = to_charger_time[k][c_id] + from_charger_time[k][c_id];
                if (detour_time_sec < INF && to_charger_energy[k][c_id] < INF && from_charger_energy[k][c_id] < INF) {
                    double r1_rate = max(1e-6, charger.r1);
                    double est_charge_time_sec = (prob.uav_params.max_battery * 0.5) / r1_rate * 3600.0;
                    candidates.push_back({detour_time_sec + est_charge_time_sec, c_id}); 
                }
            }
            sort(candidates.begin(), candidates.end());
            for (int i = 0; i < min((int)candidates.size(), K_NEAREST_CHARGERS); ++i) {
                promising_chargers_for_leg[k].push_back(candidates[i].second);
            }
        }

        if (MAX_LOCATIONS <= prob.n_tasks + prob.m_chargers + 1) {
             throw std::runtime_error("MAX_LOCATIONS constant is too small.");
        }
        
        memo_cost.assign(MAX_LOCATIONS, vector<vector<double>>(m_max_time_idx, vector<double>(m_max_charge_idx, -1.0)));
        decision_memo.assign(MAX_LOCATIONS, vector<vector<Decision>>(m_max_time_idx, vector<Decision>(m_max_charge_idx, {-2,-1})));
    }


    /**
     * @brief The core recursive DP function.
     */
    double solve_recursive(int k, int t_idx, int c_idx) {
        if (k == prob.n_tasks + 1) return 0.0; 
        
        if (t_idx >= m_max_time_idx || c_idx >= m_max_charge_idx || t_idx < 0 || c_idx < 0) return INF;
        if (memo_cost[k][t_idx][c_idx] != -1.0) return memo_cost[k][t_idx][c_idx];
        
        double current_time = t_idx * m_time_step;
        double current_charge = c_idx * m_charge_step;

        double next_release = (k < prob.n_tasks) ? prob.tasks[k].release_time : 0;
        double next_deadline = (k < prob.n_tasks) ? prob.tasks[k].deadline : INF;
        double next_exec_time = (k < prob.n_tasks) ? prob.tasks[k].exec_time : 0;

        double min_total_cost = INF;
        Decision best_decision = {-2, -1};

        // --- DECISION 1: FLY DIRECTLY ---
        double tt_direct = direct_leg_time[k];
        double flight_cost_joules = direct_leg_energy[k];
        
        if (flight_cost_joules < INF) {
            double time_arrive_next = current_time + tt_direct;
            double wait_time = max(0.0, next_release - time_arrive_next);
            double immediate_cost_joules = flight_cost_joules
                                         + (prob.uav_params.p_wt * wait_time)
                                         + (prob.uav_params.p_ex * next_exec_time);

            double charge_after_direct = current_charge - (immediate_cost_joules / 3600.0);
            if (charge_after_direct >= -1e-9) { 
                double time_depart_next = time_arrive_next + wait_time + next_exec_time;
                if (time_depart_next <= next_deadline + 1e-9) { 
                    
                    int next_t_idx = min(m_max_time_idx - 1, (int)round(time_depart_next / m_time_step));
                    int next_c_idx = max(0, min(m_max_charge_idx - 1, (int)round(charge_after_direct / m_charge_step)));
                    
                    double future_cost_joules = solve_recursive(k + 1, next_t_idx, next_c_idx);

                    if (future_cost_joules < INF) {
                        double total_direct_cost = immediate_cost_joules + future_cost_joules;
                        if (total_direct_cost < min_total_cost) {
                            min_total_cost = total_direct_cost;
                            best_decision = {-1, -1};
                        }
                    }
                }
            }
        }

        // --- DECISION 2: DETOUR VIA A PROMISING CHARGER ---
        for (int charger_id : promising_chargers_for_leg[k]) {
            const Charger& charger = prob.chargers[charger_id];
            double tt_to_cs = to_charger_time[k][charger_id];
            double energy_to_cs_joules = to_charger_energy[k][charger_id];

            double charge_at_cs = current_charge - (energy_to_cs_joules / 3600.0);
            if (charge_at_cs < -1e-9) continue; 
            charge_at_cs = max(0.0, charge_at_cs); 

            double time_at_cs = current_time + tt_to_cs;
            double tt_from_cs = from_charger_time[k][charger_id];
            double energy_from_cs_joules = from_charger_energy[k][charger_id];
            double min_charge_needed_wh = (energy_from_cs_joules / 3600.0);

            if (min_charge_needed_wh > prob.uav_params.max_battery + 1e-9) continue; 

            vector<int> target_charge_indices_to_check;
            
            // 1. Check "don't charge" action
            if (min_charge_needed_wh <= charge_at_cs + 1e-9) {
                target_charge_indices_to_check.push_back(max(0, (int)round(charge_at_cs / m_charge_step)));
            }
            // 2. Check "charge" actions
            for (double target_wh : admissible_charge_targets_wh) {
                if (target_wh > charge_at_cs + 1e-9 && target_wh >= min_charge_needed_wh - 1e-9) {
                    target_charge_indices_to_check.push_back(min(m_max_charge_idx - 1, (int)round(target_wh / m_charge_step)));
                }
            }
            sort(target_charge_indices_to_check.begin(), target_charge_indices_to_check.end());
            target_charge_indices_to_check.erase(unique(target_charge_indices_to_check.begin(), target_charge_indices_to_check.end()), target_charge_indices_to_check.end());

            for (int c_target_idx : target_charge_indices_to_check) {
                double target_charge = c_target_idx * m_charge_step;
                target_charge = min(target_charge, prob.uav_params.max_battery);

                if (target_charge < charge_at_cs - 1e-9) continue; 

                double charging_time_hours = PhysicsHelpers::calculateChargingTime(charge_at_cs, target_charge, charger, prob.uav_params);
                double time_depart_cs = time_at_cs + (charging_time_hours * 3600.0);
                double time_arrive_next = time_depart_cs + tt_from_cs;
                double wait_time_next = max(0.0, next_release - time_arrive_next);
                double time_depart_next_detour = time_arrive_next + wait_time_next + next_exec_time;

                if (time_depart_next_detour > next_deadline + 1e-9) continue; 

                double other_costs_joules = (prob.uav_params.p_wt * wait_time_next) + (prob.uav_params.p_ex * next_exec_time);
                double charge_after_detour = target_charge - ((energy_from_cs_joules + other_costs_joules) / 3600.0);

                if (charge_after_detour >= -1e-9) { 
                    
                    int next_t_idx_detour = min(m_max_time_idx - 1, (int)round(time_depart_next_detour / m_time_step));
                    int next_c_idx_detour = max(0, min(m_max_charge_idx - 1, (int)round(charge_after_detour / m_charge_step)));
                    
                    double future_cost_joules_detour = solve_recursive(k + 1, next_t_idx_detour, next_c_idx_detour);

                    if (future_cost_joules_detour < INF) {
                        double total_detour_cost = energy_to_cs_joules + energy_from_cs_joules + other_costs_joules + future_cost_joules_detour;
                        if (total_detour_cost < min_total_cost) {
                            min_total_cost = total_detour_cost;
                            best_decision = {charger_id, c_target_idx};
                        }
                    }
                }
            }
        }
        
        decision_memo[k][t_idx][c_idx] = best_decision;
        return memo_cost[k][t_idx][c_idx] = min_total_cost;
    }

    double run() {
        int initial_charge_idx = round(prob.uav_params.max_battery / m_charge_step);
        initial_charge_idx = max(0, min(m_max_charge_idx - 1, initial_charge_idx));
        return solve_recursive(0, 0, initial_charge_idx);
    }

    // --- RECONSTRUCTION WITH VERBOSE LOGGING ---
    void reconstruct_and_print_path() {
        cout << "\n===============================================" << endl;
        cout << "      OPTIMAL MISSION PATH RECONSTRUCTION      " << endl;
        cout << "===============================================" << endl;
        cout << fixed << setprecision(2); 

        int k = 0;
        int t_idx = 0;
        int c_idx = round(prob.uav_params.max_battery / m_charge_step);
        c_idx = max(0, min(m_max_charge_idx - 1, c_idx));

        while (k <= prob.n_tasks + 1) {
            if (t_idx < 0 || t_idx >= m_max_time_idx || c_idx < 0 || c_idx >= m_max_charge_idx) {
                cout << "Error: State Out of Bounds." << endl; break;
            }

            double current_time = t_idx * m_time_step;
            double current_charge = c_idx * m_charge_step;
            string location_name = (k == 0) ? "Start" : ((k > prob.n_tasks) ? "End" : "Task " + to_string(prob.tasks[k-1].id));
            
            cout << "\n----------------------------------------------------------------------------------" << endl;
            cout << "STATE: k=" << k << " (" << location_name << ") | Time=" << current_time << "s | Charge=" << current_charge << "Wh" << endl;
            cout << "----------------------------------------------------------------------------------" << endl;
            
            if (k > prob.n_tasks) {
                cout << ">>> MISSION COMPLETE <<<" << endl;
                break;
            }

            // --- 💡 ANALYSIS SECTION: WHY did we choose the action? ---
            cout << "  [ANALYSIS: Comparing Candidate Options]" << endl;

            double next_release = (k < prob.n_tasks) ? prob.tasks[k].release_time : 0;
            double next_deadline = (k < prob.n_tasks) ? prob.tasks[k].deadline : INF;
            double next_exec_time = (k < prob.n_tasks) ? prob.tasks[k].exec_time : 0;
            
            Decision best_dec = decision_memo[k][t_idx][c_idx];

            // 1. Analyze "Fly Direct"
            {
                double tt = direct_leg_time[k];
                double cost_J = direct_leg_energy[k];
                double total = INF;
                string status = "Pruned (Physics)";
                
                if (cost_J < INF) {
                    double t_arr = current_time + tt;
                    double t_wait = max(0.0, next_release - t_arr);
                    double imm_cost = cost_J + (prob.uav_params.p_wt * t_wait) + (prob.uav_params.p_ex * next_exec_time);
                    double ch_rem = current_charge - (imm_cost / 3600.0);
                    
                    if (ch_rem >= -1e-9) {
                        double t_dep = t_arr + t_wait + next_exec_time;
                        if (t_dep <= next_deadline + 1e-9) {
                            int nt = min(m_max_time_idx - 1, (int)round(t_dep / m_time_step));
                            int nc = max(0, min(m_max_charge_idx - 1, (int)round(ch_rem / m_charge_step)));
                            
                            // Lookup future cost (memoized)
                            double future = (k+1 > prob.n_tasks) ? 0.0 : ((memo_cost[k+1][nt][nc] == -1) ? INF : memo_cost[k+1][nt][nc]);
                            
                            if (future < INF) {
                                total = imm_cost + future;
                                status = "Feasible";
                            } else status = "Pruned (Future Impossible)";
                        } else status = "Pruned (Deadline)";
                    } else status = "Pruned (Battery)";
                }

                bool is_chosen = (best_dec.action_type == -1);
                cout << "    > Option [Direct Flight]: " << setw(10) << (total >= INF ? "INF" : to_string(total)) 
                     << " J " << (is_chosen ? "<<< CHOSEN" : "") << " (" << status << ")" << endl;
            }

            // 2. Analyze "Charger Detours"
            for (int cid : promising_chargers_for_leg[k]) {
                 double tt_to = to_charger_time[k][cid];
                 double en_to = to_charger_energy[k][cid];
                 double ch_at_cs = max(0.0, current_charge - (en_to/3600.0));
                 
                 // Re-derive target indices (same logic as solve_recursive)
                 double tt_from = from_charger_time[k][cid];
                 double en_from = from_charger_energy[k][cid];
                 double min_need = en_from / 3600.0;
                 
                 vector<int> targs;
                 if (min_need <= ch_at_cs + 1e-9) targs.push_back(max(0, (int)round(ch_at_cs / m_charge_step)));
                 for (double twh : admissible_charge_targets_wh) {
                     if (twh > ch_at_cs + 1e-9 && twh >= min_need - 1e-9) 
                        targs.push_back(min(m_max_charge_idx - 1, (int)round(twh / m_charge_step)));
                 }
                 sort(targs.begin(), targs.end());
                 targs.erase(unique(targs.begin(), targs.end()), targs.end());

                 for (int t_idx_target : targs) {
                     double targ_wh = min((double)t_idx_target * m_charge_step, prob.uav_params.max_battery);
                     double charge_dur = PhysicsHelpers::calculateChargingTime(ch_at_cs, targ_wh, prob.chargers[cid], prob.uav_params) * 3600.0;
                     double t_dep_cs = current_time + tt_to + charge_dur;
                     double t_arr_next = t_dep_cs + tt_from;
                     double t_wait = max(0.0, next_release - t_arr_next);
                     double t_dep_next = t_arr_next + t_wait + next_exec_time;
                     
                     double total = INF;
                     string status = "Feasible";

                     if (t_dep_next > next_deadline + 1e-9) {
                         status = "Pruned (Deadline)";
                     } else {
                         double other = (prob.uav_params.p_wt * t_wait) + (prob.uav_params.p_ex * next_exec_time);
                         double ch_rem = targ_wh - ((en_from + other)/3600.0);
                         
                         int nt = min(m_max_time_idx - 1, (int)round(t_dep_next / m_time_step));
                         int nc = max(0, min(m_max_charge_idx - 1, (int)round(ch_rem / m_charge_step)));
                         
                         double future = (k+1 > prob.n_tasks) ? 0.0 : ((memo_cost[k+1][nt][nc] == -1) ? INF : memo_cost[k+1][nt][nc]);
                         
                         if (future < INF) total = en_to + en_from + other + future;
                         else status = "Pruned (Future Impossible)";
                     }

                     bool is_chosen = (best_dec.action_type == cid && best_dec.target_charge_idx == t_idx_target);
                     cout << "    > Option [Charger " << setw(2) << cid << " @ " << setw(5) << targ_wh << "Wh]: " 
                          << setw(10) << (total >= INF ? "INF" : to_string(total)) 
                          << " J " << (is_chosen ? "<<< CHOSEN" : "") << " (" << status << ")" << endl;
                 }
            }
            cout << endl;

            // --- EXECUTE CHOSEN ACTION (Update State) ---
            if (best_dec.action_type == -2) { cout << "  -> FAIL: No valid decision." << endl; break; }
            
            double next_loc_exec_time = (k < prob.n_tasks) ? prob.tasks[k].exec_time : 0;
            string next_loc_name = (k < prob.n_tasks) ? ("Task " + to_string(prob.tasks[k].id)) : "End Depot";

            if (best_dec.action_type == -1) {
                // Direct
                double tt = direct_leg_time[k];
                double cost = direct_leg_energy[k];
                double t_arr = current_time + tt;
                double t_wait = max(0.0, next_release - t_arr);
                double t_dep = t_arr + t_wait + next_loc_exec_time;
                double tot_cost_leg = cost + (prob.uav_params.p_wt * t_wait) + (prob.uav_params.p_ex * next_loc_exec_time);
                
                cout << "  >>> ACTION TAKEN: Fly Direct to " << next_loc_name << endl;
                cout << "      Travel Time: " << tt << "s | Wait: " << t_wait << "s | Leg Cost: " << tot_cost_leg << " J" << endl;
                
                double new_ch = current_charge - (tot_cost_leg / 3600.0);
                t_idx = min(m_max_time_idx - 1, (int)round(t_dep / m_time_step));
                c_idx = max(0, min(m_max_charge_idx - 1, (int)round(new_ch / m_charge_step)));
            } else {
                // Charger
                int cid = best_dec.action_type;
                int targs_idx = best_dec.target_charge_idx;
                double targ_wh = min((double)targs_idx * m_charge_step, prob.uav_params.max_battery);
                
                double tt_to = to_charger_time[k][cid];
                double en_to = to_charger_energy[k][cid];
                double ch_at = max(0.0, current_charge - (en_to/3600.0));
                double dur = PhysicsHelpers::calculateChargingTime(ch_at, targ_wh, prob.chargers[cid], prob.uav_params) * 3600.0;
                double t_dep_cs = current_time + tt_to + dur;
                
                double tt_from = from_charger_time[k][cid];
                double en_from = from_charger_energy[k][cid];
                double t_arr_next = t_dep_cs + tt_from;
                double t_wait = max(0.0, next_release - t_arr_next);
                double other_cost = (prob.uav_params.p_wt * t_wait) + (prob.uav_params.p_ex * next_loc_exec_time);
                
                cout << "  >>> ACTION TAKEN: Detour via Charger " << cid << endl;
                cout << "      Leg 1 (To CS): " << tt_to << "s, " << en_to << " J" << endl;
                cout << "      Charging: " << ch_at << " -> " << targ_wh << " Wh (" << dur << "s)" << endl;
                cout << "      Leg 2 (Fr CS): " << tt_from << "s, " << (en_from + other_cost) << " J" << endl;
                
                double final_ch = targ_wh - ((en_from + other_cost)/3600.0);
                double t_final = t_arr_next + t_wait + next_loc_exec_time;
                
                t_idx = min(m_max_time_idx - 1, (int)round(t_final / m_time_step));
                c_idx = max(0, min(m_max_charge_idx - 1, (int)round(final_ch / m_charge_step)));
            }
            k++;
        }
    }
};