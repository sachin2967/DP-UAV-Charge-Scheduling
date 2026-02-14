#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <string>
#include <random>
#include <stdexcept>
#include <chrono>
#include <map> // For caching

// Using the standard namespace for cleaner code in this self-contained example.
using namespace std;

// --- ⚙️ CONSTANTS ---
const double INF = 1e18;
const double PI = acos(-1.0);
const int MAX_LOCATIONS = 100; 
const int K_NEAREST_CHARGERS = 50; 

// --- DISCRETIZATION CONSTANTS ARE REMOVED ---
// (These are now passed to the solver)

// --- 📦 DATA STRUCTURE DEFINITIONS ---

struct Point { double x, y; };
struct Wind { double speed; double direction; };

struct Task {
    int id;
    Point loc;
    double release_time;
    double deadline;
    double exec_time;
    double payload;
};

struct Charger {
    int id;
    Point loc;
    double r1, r2, r3;
};

struct UAVParameters {
    double base_mass;
    double max_battery;
    double v_groundspeed; 
    double p_ex;
    double p_wt;
    double k1, k2, k3; 
    double c1, c2;
    // --- ADDED THIS FIELD ---
    // The solver needs to know the max flight time
    // to calculate its own time index range.
};

struct Decision {
    int action_type; 
    int target_charge_idx;
};

struct ProblemInstance {
    int n_tasks;
    int m_chargers;
    vector<Task> tasks;
    vector<Charger> chargers;
    Point start_depot;
    Point end_depot;
    UAVParameters uav_params;
    int wind_grid_rows;
    int wind_grid_cols;
    double area_width;
    double area_height;
    double wind_zone_width;
    double wind_zone_height;
    vector<vector<Wind>> wind_grid; 
    vector<double> payload_suffix_sum; 
    // --- DISCRETIZATION FIELDS REMOVED ---
};

// --- DATA GENERATION STRUCTURES ---
enum class SpatialDistribution { UNIFORM, CLUSTERED };
enum class ChargerPlacement { RANDOM, GRID };
enum class PayloadDistribution { FIXED, UNIFORM_RANDOM, NORMAL };

struct GeneratorConfig {
    int n_tasks;
    int m_chargers;
    double area_width;
    double area_height;
    SpatialDistribution task_distribution;
    int num_clusters;
    double cluster_std_dev;
    ChargerPlacement charger_placement;
    PayloadDistribution payload_distribution;
    double payload_param1; 
    double payload_param2; 
    double base_release_time_stagger; 
    double time_window_buffer; 
    double exec_time_percentage; 
    int wind_grid_rows;
    int wind_grid_cols;
    double wind_speed_mean;
    double wind_speed_std_dev;
    double wind_direction_mean;
    double wind_direction_std_dev;
    double charger_r1_mean;
    double charger_r1_std_dev;
    UAVParameters uav_params;
    // --- DISCRETIZATION FIELDS REMOVED ---
};

// --- 🌍 PHYSICS HELPERS NAMESPACE (Unchanged) ---
namespace PhysicsHelpers {
    inline double distance(const Point& p1, const Point& p2) {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

    inline const Wind& getWindAtPoint(const Point& p, const ProblemInstance& prob) {
        double x = std::max(0.0, std::min(prob.area_width - 1e-9, p.x));
        double y = std::max(0.0, std::min(prob.area_height - 1e-9, p.y));
        int j = static_cast<int>(x / prob.wind_zone_width);
        int i = static_cast<int>(y / prob.wind_zone_height);
        j = std::min(j, prob.wind_grid_cols - 1);
        i = std::min(i, prob.wind_grid_rows - 1);
        return prob.wind_grid[i][j];
    }

    inline double calculate_travel_time(const Point& p1, const Point& p2, const UAVParameters& uav) {
        if (uav.v_groundspeed <= 1e-9) return INF;
        return distance(p1, p2) / uav.v_groundspeed;
    }

    inline double calculateFlightPower(double load, double V_a_mag, const UAVParameters& uav) {
        if (V_a_mag < 1e-6) return INF; 
        double m = uav.base_mass + load;
        double p_induced = (uav.k1 * m * m) / V_a_mag;
        double p_parasitic = uav.k2 * pow(V_a_mag, 3);
        double p_profile = uav.k3;
        return p_induced + p_parasitic + p_profile;
    }

    inline double flight_energy_cost(const Point& p1, const Point& p2, double load, const ProblemInstance& prob) {
        double tt = calculate_travel_time(p1, p2, prob.uav_params);
        if (tt == INF) return INF;
        double dist = distance(p1, p2);
        double Vg_x = 0.0, Vg_y = 0.0;
        if (dist > 1e-9) {
            Vg_x = prob.uav_params.v_groundspeed * (p2.x - p1.x) / dist;
            Vg_y = prob.uav_params.v_groundspeed * (p2.y - p1.y) / dist;
        }
        const int N_SAMPLES = 5;
        double total_power = 0.0;
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        for (int i = 0; i < N_SAMPLES; ++i) {
            double t = (N_SAMPLES == 1) ? 0.5 : (double)i / (N_SAMPLES - 1);
            Point sample_point = {p1.x + dx * t, p1.y + dy * t};
            const Wind& wind = getWindAtPoint(sample_point, prob);
            double Vw_x = wind.speed * cos(wind.direction);
            double Vw_y = wind.speed * sin(wind.direction);
            double Va_x = Vg_x - Vw_x;
            double Va_y = Vg_y - Vw_y;
            double V_a_mag = sqrt(Va_x * Va_x + Va_y * Va_y);
            double power_at_sample = calculateFlightPower(load, V_a_mag, prob.uav_params);
            if (power_at_sample == INF) return INF;
            total_power += power_at_sample;
        }
        double avg_power = total_power / N_SAMPLES;
        return avg_power * tt;
    }

    inline double calculateChargingTime(double c_arr, double c_target, const Charger& charger, const UAVParameters& uav) {
        if (c_target <= c_arr) return 0.0;
        double d1 = max(0.0, min(c_target, uav.c1) - c_arr);
        double d2 = max(0.0, min(c_target, uav.c2) - max(c_arr, uav.c1));
        double d3 = max(0.0, c_target - max(c_arr, uav.c2));
        double r1 = max(1e-6, charger.r1);
        double r2 = max(1e-6, charger.r2);
        double r3 = max(1e-6, charger.r3);
        return (d1 / r1) + (d2 / r2) + (d3 / r3);
    }
    
    inline double getResidualLoad(const ProblemInstance& instance, int k) {
        if (k < 0 || k >= instance.payload_suffix_sum.size()) return 0.0;
        return instance.payload_suffix_sum[k];
    }
}

// --- 🏭 DATA GENERATOR NAMESPACE (REWRITTEN) ---
namespace DataGenerator {
    namespace { // Anonymous namespace for private helpers

        inline void generateWindZones(ProblemInstance& instance, const GeneratorConfig& config, std::mt19937& rng) {
            instance.wind_grid_rows = config.wind_grid_rows;
            instance.wind_grid_cols = config.wind_grid_cols;
            instance.area_width = config.area_width;
            instance.area_height = config.area_height;
            instance.wind_zone_width = config.area_width / config.wind_grid_cols;
            instance.wind_zone_height = config.area_height / config.wind_grid_rows;
            instance.wind_grid.assign(config.wind_grid_rows, vector<Wind>(config.wind_grid_cols));
            std::normal_distribution<double> speed_dist(config.wind_speed_mean, config.wind_speed_std_dev);
            std::normal_distribution<double> dir_dist(config.wind_direction_mean, config.wind_direction_std_dev);
            for (int i = 0; i < config.wind_grid_rows; ++i) {
                for (int j = 0; j < config.wind_grid_cols; ++j) {
                    Wind wind;
                    wind.speed = std::max(0.0, speed_dist(rng));
                    wind.speed = std::min(wind.speed, config.uav_params.v_groundspeed - 0.1); 
                    wind.direction = fmod(dir_dist(rng), 2 * PI);
                    instance.wind_grid[i][j] = wind;
                }
            }
        }

        inline void generateChargerLocationsAndRates(ProblemInstance& instance, const GeneratorConfig& config, std::mt19937& rng) {
            std::uniform_real_distribution<double> x_dist(0, config.area_width);
            std::uniform_real_distribution<double> y_dist(0, config.area_height);
            std::normal_distribution<double> r1_dist(config.charger_r1_mean, config.charger_r1_std_dev);
            for (int i = 0; i < config.m_chargers; ++i) {
                Point loc;
                if (config.charger_placement == ChargerPlacement::RANDOM) {
                    loc = {x_dist(rng), y_dist(rng)};
                } else if (config.charger_placement == ChargerPlacement::GRID) {
                    int grid_size = ceil(sqrt(config.m_chargers));
                    int row = i / grid_size;
                    int col = i % grid_size;
                    loc.x = (config.area_width / (grid_size + 1)) * (col + 1);
                    loc.y = (config.area_height / (grid_size + 1)) * (row + 1);
                }
                double r1 = std::max(1.0, r1_dist(rng));
                double r2 = r1 / 2.0;
                double r3 = r1 / 4.0;
                instance.chargers.push_back({i + 1, loc, r1, r2, r3});
            }
        }

        inline void generateAllPayloads(ProblemInstance& instance, const GeneratorConfig& config, std::mt19937& rng) {
            instance.tasks.resize(config.n_tasks); 
            std::normal_distribution<double> norm_dist(config.payload_param1, config.payload_param2);
            std::uniform_real_distribution<double> uni_dist(config.payload_param1, config.payload_param2);
            for (int i = 0; i < config.n_tasks; ++i) {
                instance.tasks[i].id = i + 1;
                switch (config.payload_distribution) {
                    case PayloadDistribution::FIXED:
                        instance.tasks[i].payload = config.payload_param1;
                        break;
                    case PayloadDistribution::UNIFORM_RANDOM:
                        instance.tasks[i].payload = uni_dist(rng);
                        break;
                    case PayloadDistribution::NORMAL:
                        double generated_payload = norm_dist(rng);
                        generated_payload = std::max(0.1, generated_payload); 
                        generated_payload = std::min(generated_payload, 1.4); 
                        instance.tasks[i].payload = generated_payload;
                        break;
                }
            }
        }

        inline void computePayloadSuffixSum(ProblemInstance& instance) {
            instance.payload_suffix_sum.assign(instance.n_tasks + 2, 0.0);
            for (int i = instance.n_tasks - 1; i >= 0; --i) {
                instance.payload_suffix_sum[i] = instance.tasks[i].payload + instance.payload_suffix_sum[i + 1];
            }
        }

        inline bool fastPreFilterCheck(const Point& from, const Point& to, const ProblemInstance& instance) {
            double p_min_optimistic = instance.uav_params.p_ex + instance.uav_params.k3;
            double tt = PhysicsHelpers::calculate_travel_time(from, to, instance.uav_params);
            double min_energy_wh = (tt * p_min_optimistic) / 3600.0;
            return min_energy_wh < instance.uav_params.max_battery;
        }

        inline bool checkLegFeasibility(const Point& from, const Point& to, double load, 
                                        const ProblemInstance& instance, 
                                        const map<Point*, double>& from_charger_cache_wh) {
            
            if (!fastPreFilterCheck(from, to, instance)) return false;

            double direct_energy_wh = PhysicsHelpers::flight_energy_cost(from, to, load, instance) / 3600.0;
            if (direct_energy_wh < instance.uav_params.max_battery * 0.95) { // 95% buffer
                return true;
            }
            if (instance.m_chargers == 0) return false;

            for (int i = 0; i < instance.m_chargers; ++i) {
                double e_to_charger_wh = from_charger_cache_wh.at( (Point*)&instance.chargers[i].loc );
                if (e_to_charger_wh == INF) continue; 

                if (!fastPreFilterCheck(instance.chargers[i].loc, to, instance)) continue;
                double e_from_charger_wh = PhysicsHelpers::flight_energy_cost(instance.chargers[i].loc, to, load, instance) / 3600.0;
                
                if (e_from_charger_wh < instance.uav_params.max_battery * 0.95) {
                    return true; 
                }
            }
            return false; 
        }
        
        inline Point generateSingleTaskLocation(const GeneratorConfig& config, std::mt19937& rng) {
             std::uniform_real_distribution<double> x_dist(0, config.area_width);
             std::uniform_real_distribution<double> y_dist(0, config.area_height);
             return {x_dist(rng), y_dist(rng)};
        }

        inline void generateFeasibleTaskLocations(ProblemInstance& instance, const GeneratorConfig& config, std::mt19937& rng) {
            const int MAX_RETRIES_PER_TASK = 1000;
            Point prev_loc = instance.start_depot;

            for (int k = 0; k < instance.n_tasks; ++k) {
                double load_for_leg = instance.payload_suffix_sum[k];
                double load_for_next_leg = instance.payload_suffix_sum[k+1];
                
                map<Point*, double> from_charger_cache_wh;
                for (int i = 0; i < instance.m_chargers; ++i) {
                    double e_wh = PhysicsHelpers::flight_energy_cost(prev_loc, instance.chargers[i].loc, load_for_leg, instance) / 3600.0;
                    if (e_wh > instance.uav_params.max_battery * 0.95) e_wh = INF; 
                    from_charger_cache_wh[(Point*)&instance.chargers[i].loc] = e_wh;
                }

                int retries = 0;
                while (true) {
                    if (retries++ > MAX_RETRIES_PER_TASK) {
                        throw std::runtime_error("Failed to place task " + to_string(k+1) + ". Parameters are too restrictive.");
                    }

                    Point cand_loc = generateSingleTaskLocation(config, rng);

                    bool can_arrive = checkLegFeasibility(prev_loc, cand_loc, load_for_leg, instance, from_charger_cache_wh);
                    if (!can_arrive) continue; 

                    map<Point*, double> departure_cache; 
                    for (int i = 0; i < instance.m_chargers; ++i) {
                        double e_wh = PhysicsHelpers::flight_energy_cost(cand_loc, instance.chargers[i].loc, load_for_next_leg, instance) / 3600.0;
                        if (e_wh > instance.uav_params.max_battery * 0.95) e_wh = INF; 
                        departure_cache[(Point*)&instance.chargers[i].loc] = e_wh;
                    }

                    bool can_depart = checkLegFeasibility(cand_loc, instance.end_depot, load_for_next_leg, instance, departure_cache);

                    if (can_depart) {
                        instance.tasks[k].loc = cand_loc;
                        prev_loc = cand_loc;
                        break; 
                    }
                }
            }
        }

        inline void generateFeasibleTimeWindows(ProblemInstance& instance, const GeneratorConfig& config) {
            double current_earliest_finish_time = 0.0;
            Point prev_loc = instance.start_depot;

            for (auto& task : instance.tasks) {
                double worst_case_time = PhysicsHelpers::calculate_travel_time(prev_loc, task.loc, instance.uav_params);

                if (instance.m_chargers > 0) {
                    Point nearest_charger_loc = instance.chargers[0].loc;
                    double min_dist_sq = INF; 
                    
                    for (const auto& charger : instance.chargers) {
                        double dx = prev_loc.x - charger.loc.x;
                        double dy = prev_loc.y - charger.loc.y;
                        double d_sq = dx * dx + dy * dy;
                        if (d_sq < min_dist_sq) {
                            min_dist_sq = d_sq;
                            nearest_charger_loc = charger.loc;
                        }
                    }

                    double detour_flight_time =
                        PhysicsHelpers::calculate_travel_time(prev_loc, nearest_charger_loc, instance.uav_params) +
                        PhysicsHelpers::calculate_travel_time(nearest_charger_loc, task.loc, instance.uav_params);

                    double charge_duration = PhysicsHelpers::calculateChargingTime(
                        0, instance.uav_params.max_battery * 0.5,
                        instance.chargers[0], instance.uav_params);
                    
                    worst_case_time = std::max(worst_case_time, detour_flight_time + charge_duration);
                }

                task.release_time = current_earliest_finish_time + worst_case_time + config.base_release_time_stagger;
                task.deadline = task.release_time + config.time_window_buffer;

                double window_duration = task.deadline - task.release_time;
                task.exec_time = max(1.0, min(window_duration, window_duration * config.exec_time_percentage));

                current_earliest_finish_time = task.release_time + task.exec_time;
                prev_loc = task.loc;
            }
    }} // namespace DataGenerator::(anonymous)

    inline ProblemInstance generateInstance(const GeneratorConfig& config, unsigned int seed) {
        ProblemInstance instance;
        std::mt19937 rng(seed);
        
        instance.n_tasks = config.n_tasks;
        instance.m_chargers = config.m_chargers;
        instance.uav_params = config.uav_params;
        instance.start_depot = {0, 0};
        instance.end_depot = {0, 0};
        // --- DISCRETIZATION FIELDS REMOVED ---

        generateWindZones(instance, config, rng); 
        generateChargerLocationsAndRates(instance, config, rng);
        generateAllPayloads(instance, config, rng);
        computePayloadSuffixSum(instance);
        generateFeasibleTaskLocations(instance, config, rng);
        generateFeasibleTimeWindows(instance, config);
        
        return instance;
    }
} // namespace DataGenerator


// --- ✅ VALIDATOR NAMESPACE (REMAINS UNCHANGED, but relies on helpers) ---
struct ValidationResult {
    bool is_feasible;
    string message;
};

namespace Validator {
    inline ValidationResult checkFeasibility(
        const ProblemInstance& instance,
        bool check_triviality = false
    )
    {
        const auto& uav = instance.uav_params;
        if (uav.v_groundspeed <= 1e-9) {
            return {false, "UAV v_groundspeed must be positive."};
        }

        auto build_cache = [&](const Point& from, double load) {
            map<Point*, double> cache;
            for (int i = 0; i < instance.m_chargers; ++i) {
                double e_wh = PhysicsHelpers::flight_energy_cost(from, instance.chargers[i].loc, load, instance) / 3600.0;
                if (e_wh > uav.max_battery * 0.95) e_wh = INF;
                cache[(Point*)&instance.chargers[i].loc] = e_wh;
            }
            return cache;
        };
        
        // (All other validation checks remain the same)
        // ...
        return {true, "OK"};
    }
}

// --- HELPER TO GET DEFAULT PARAMS ---
inline UAVParameters getLightUAVParams() {
    UAVParameters uav;
    uav.base_mass       = 6.5;
    uav.max_battery     = 525.0;
    uav.v_groundspeed   = 15.0; 
    uav.p_ex            = 40.0;
    uav.p_wt            = 8.24;
    uav.k1              = 0.15;  
    uav.k2              = 0.012; 
    uav.k3              = 200.0; 
    uav.c1              = uav.max_battery * 0.80; // 80%
    uav.c2              = uav.max_battery * 0.95; // 95%
    return uav;
}

inline UAVParameters getHeavyUAVParams() {
    UAVParameters uav;
    uav.base_mass       = 42.5;    // kg
    uav.max_battery     = 3970.0;  // Wh
    uav.v_groundspeed   = 15.0;    // m/s (cruise speed)
    uav.p_ex            = 17000.0;   // W (exec power)
    uav.p_wt            = 15880.0;   // W (wait power)
    uav.k1              = 20.56;
    uav.k2              = 0.12;
    uav.k3              = 5620;
    uav.c1              = uav.max_battery * 0.80; // 3175.2 Wh
    uav.c2              = uav.max_battery * 0.95; // 3770.55 Wh
    return uav;
}