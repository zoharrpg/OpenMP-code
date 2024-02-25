/**
 * Parallel VLSI Wire Routing via OpenMP
 * Name 1(andrew_id 1), Name 2(andrew_id 2)
 */

#include "wireroute.h"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <string>
#include <vector>

#include <unistd.h>
#include <omp.h>
#include <random>

void print_stats(const std::vector<std::vector<int>> &occupancy) {
    int max_occupancy = 0;
    long long total_cost = 0;

    for (const auto &row: occupancy) {
        for (const int count: row) {
            max_occupancy = std::max(max_occupancy, count);
            total_cost += count * count;
        }
    }

    std::cout << "Max occupancy: " << max_occupancy << '\n';
    std::cout << "Total cost: " << total_cost << '\n';
}

void write_output(const std::vector<Wire> &wires, const int num_wires, const std::vector<std::vector<int>> &occupancy,
                  const int dim_x, const int dim_y, const int num_threads, std::string input_filename) {
    if (std::size(input_filename) >= 4 && input_filename.substr(std::size(input_filename) - 4) == ".txt") {
        input_filename.resize(std::size(input_filename) - 4);
    }

    const std::string occupancy_filename = input_filename + "_occupancy_" + std::to_string(num_threads) + ".txt";
    const std::string wires_filename = input_filename + "_wires_" + std::to_string(num_threads) + ".txt";

    std::ofstream out_occupancy(occupancy_filename, std::fstream::out);
    if (!out_occupancy) {
        std::cerr << "Unable to open file: " << occupancy_filename << '\n';
        exit(EXIT_FAILURE);
    }

    out_occupancy << dim_x << ' ' << dim_y << '\n';
    for (const auto &row: occupancy) {
        for (const int count: row) {
            out_occupancy << count << ' ';
        }
        out_occupancy << '\n';
    }

    out_occupancy.close();

    std::ofstream out_wires(wires_filename, std::fstream::out);
    if (!out_wires) {
        std::cerr << "Unable to open file: " << wires_filename << '\n';
        exit(EXIT_FAILURE);
    }

    out_wires << dim_x << ' ' << dim_y << '\n' << num_wires << '\n';

    for (const auto &[start_x, start_y, end_x, end_y, bend1_x, bend1_y]: wires) {
        out_wires << start_x << ' ' << start_y << ' ' << bend1_x << ' ' << bend1_y << ' ';

        if (start_y == bend1_y) {
            // first bend was horizontal

            if (end_x != bend1_x) {
                // two bends

                out_wires << bend1_x << ' ' << end_y << ' ';
            }
        } else if (start_x == bend1_x) {
            // first bend was vertical

            if (end_y != bend1_y) {
                // two bends

                out_wires << end_x << ' ' << bend1_y << ' ';
            }
        }
        out_wires << end_x << ' ' << end_y << '\n';
    }

    out_wires.close();
}

double fixed_probability(const double prob = -1) {
    static double fixed_prob = -1;
    if (prob > 0) {
        fixed_prob = prob;
    }
    return fixed_prob;
}

bool does_happen() {
    static thread_local std::mt19937 generator(std::random_device{}());
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    static const double probability = fixed_probability();
    return distribution(generator) < probability;
}

void random_bend(Wire &wire) {
    int delta_x = std::abs(wire.end_x - wire.start_x);
    int delta_y = std::abs(wire.end_y - wire.start_y);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis_x(1, delta_x + delta_y);

    // 6,1
    // 3,5
    int random = dis_x(gen);
    if (random <= delta_x) {
        wire.bend1_x = std::min(wire.start_x, wire.end_x) + random;
        wire.bend1_y = wire.start_y;
    } else {
        wire.bend1_x = wire.end_x;
        wire.bend1_y = std::min(wire.start_y, wire.end_y) + random - delta_x;
    }
}

int main(int argc, char *argv[]) {
    const auto init_start = std::chrono::steady_clock::now();

    std::string input_filename;
    int num_threads = 0;
    double SA_prob = 0.1;
    int SA_iters = 5;
    char parallel_mode = '\0';
    int batch_size = 1;

    int opt;
    while ((opt = getopt(argc, argv, "f:n:p:i:m:b:")) != -1) {
        switch (opt) {
            case 'f':
                input_filename = optarg;
                break;
            case 'n':
                num_threads = atoi(optarg);
                break;
            case 'p':
                SA_prob = atof(optarg);
                break;
            case 'i':
                SA_iters = atoi(optarg);
                break;
            case 'm':
                parallel_mode = *optarg;
                break;
            case 'b':
                batch_size = atoi(optarg);
                break;
            default:
                std::cerr << "Usage: " << argv[0]
                          << " -f input_filename -n num_threads [-p SA_prob] [-i SA_iters] -m parallel_mode -b batch_size\n";
                exit(EXIT_FAILURE);
        }
    }

    // Check if required options are provided
    if (empty(input_filename) || num_threads <= 0 || SA_iters <= 0 || (parallel_mode != 'A' && parallel_mode != 'W') ||
        batch_size <= 0) {
        std::cerr << "Usage: " << argv[0]
                  << " -f input_filename -n num_threads [-p SA_prob] [-i SA_iters] -m parallel_mode -b batch_size\n";
        exit(EXIT_FAILURE);
    }

    std::cout << "Number of threads: " << num_threads << '\n';
    std::cout << "Simulated annealing probability parameter: " << SA_prob << '\n';
    std::cout << "Simulated annealing iterations: " << SA_iters << '\n';
    std::cout << "Input file: " << input_filename << '\n';
    std::cout << "Parallel mode: " << parallel_mode << '\n';
    std::cout << "Batch size: " << batch_size << '\n';

    std::ifstream fin(input_filename);

    if (!fin) {
        std::cerr << "Unable to open file: " << input_filename << ".\n";
        exit(EXIT_FAILURE);
    }

    int dim_x, dim_y;
    int num_wires;

    /* Read the grid dimension and wire information from file */
    fin >> dim_x >> dim_y >> num_wires;

    std::vector<Wire> wires(num_wires);
    std::vector occupancy(dim_y, std::vector<int>(dim_x));

    for (auto &wire: wires) {
        fin >> wire.start_x >> wire.start_y >> wire.end_x >> wire.end_y;
        wire.bend1_x = wire.start_x;
        wire.bend1_y = wire.end_y;
    }

    /* Initialize any additional data structures needed in the algorithm */
    initialize(wires, occupancy);
    const double init_time = std::chrono::duration_cast<std::chrono::duration<
            double >>(std::chrono::steady_clock::now() - init_start).count();
    fixed_probability(SA_prob);
    std::cout << "Initialization time (sec): " << std::fixed << std::setprecision(10) << init_time << '\n';

    const auto compute_start = std::chrono::steady_clock::now();

    /**
     * Implement the wire routing algorithm here
     * Feel free to structure the algorithm into different functions
     * Don't use global variables.
     * Use OpenMP to parallelize the algorithm.
     */
    omp_set_num_threads(num_threads);
    switch (parallel_mode) {
        case 'W':
            within_wires(wires, occupancy, SA_iters);
            break;
        case 'A':
            break;
    }

    const double compute_time = std::chrono::duration_cast<std::chrono::duration<
            double >>(std::chrono::steady_clock::now() - compute_start).count();
    std::cout << "Computation time (sec): " << compute_time << '\n';

    /* Write wires and occupancy matrix to files */

    print_stats(occupancy);
    write_output(wires, num_wires, occupancy, dim_x, dim_y, num_threads, input_filename);
}

validate_wire_t Wire::to_validate_format(void) const {
    validate_wire_t wire{};
    wire.p[0].x = start_x;
    wire.p[0].y = start_y;

    if (int num_bend = num_bends(*this); 0 == num_bend) {
        wire.p[1].x = end_x;
        wire.p[1].y = end_y;
    } else if (num_bend == 1) {
        wire.p[1].x = bend1_x;
        wire.p[1].y = bend1_y;
        wire.p[2].x = end_x;
        wire.p[2].y = end_y;
    } else {
        wire.p[1].x = bend1_x;
        wire.p[1].y = bend1_y;
        wire.p[3].x = end_x;
        wire.p[3].y = end_y;
        if (bend1_x == start_x) {
            wire.p[2].x = end_x;
            wire.p[2].y = bend1_y;
        } else {
            wire.p[2].x = bend1_x;
            wire.p[2].y = end_x;
        }
    }
    return wire;
}

int num_bends(const Wire &wire) {
    /* Returns the number of bends in the wire */
    return (wire.start_x != wire.bend1_x) + (wire.bend1_x != wire.end_x) + (wire.start_y != wire.bend1_y) +
           (wire.bend1_y != wire.end_y) - 1;
}

template<bool CalculateDeltaCost, bool UpdateOccupancy>
cost_t update_point(const int x, const int y, std::vector<std::vector<int>> &occupancy, const int delta) {
    /* Update the occupancy count of a point (x, y) */
    cost_t delta_cost = 0;
    if constexpr (CalculateDeltaCost) {
        delta_cost = (occupancy[y][x] + delta) * (occupancy[y][x] + delta) - occupancy[y][x] * occupancy[y][x];
    }
    if constexpr (UpdateOccupancy) {
        occupancy[y][x] += delta;
    }
    return delta_cost;
}

template<bool CalculateDeltaCost, bool UpdateOccupancy>
cost_t update_wire_no_bend(const Wire &wire, std::vector<std::vector<int>> &occupancy, const int delta) {
    cost_t delta_cost = 0;
    if (wire.start_y == wire.end_y) {
        // Horizontal wire
        for (int x = std::min(wire.start_x, wire.end_x); x <= std::max(wire.start_x, wire.end_x); x++) {
            delta_cost += update_point<CalculateDeltaCost, UpdateOccupancy>(x, wire.start_y, occupancy, delta);
        }
    } else {
        // Vertical wire
        for (int y = std::min(wire.start_y, wire.end_y); y <= std::max(wire.start_y, wire.end_y); y++) {
            delta_cost += update_point<CalculateDeltaCost, UpdateOccupancy>(wire.start_x, y, occupancy, delta);
        }
    }
    return delta_cost;
}

template<bool CalculateDeltaCost, bool UpdateOccupancy>
cost_t update_wire_one_bend(const Wire &wire, std::vector<std::vector<int>> &occupancy, const int delta) {
    cost_t delta_cost = 0;
    if (wire.start_x == wire.bend1_x) {
        // Vertical first bend
        for (int y = std::min(wire.start_y, wire.bend1_y); y < std::max(wire.start_y, wire.bend1_y); y++) {
            delta_cost += update_point<CalculateDeltaCost, UpdateOccupancy>(wire.start_x, y, occupancy, delta);
        }

        // Horizontal second bend
        for (int x = std::min(wire.bend1_x, wire.end_x); x <= std::max(wire.bend1_x, wire.end_x); x++) {
            delta_cost += update_point<CalculateDeltaCost, UpdateOccupancy>(x, wire.bend1_y, occupancy, delta);
        }
    } else {
        // Horizontal first bend
        for (int x = std::min(wire.start_x, wire.bend1_x); x < std::max(wire.start_x, wire.bend1_x); x++) {
            delta_cost += update_point<CalculateDeltaCost, UpdateOccupancy>(x, wire.start_y, occupancy, delta);
        }

        // Vertical second bend
        for (int y = std::min(wire.bend1_y, wire.end_y); y <= std::max(wire.bend1_y, wire.end_y); y++) {
            delta_cost += update_point<CalculateDeltaCost, UpdateOccupancy>(wire.bend1_x, y, occupancy, delta);
        }
    }

    return delta_cost;
}

template<bool CalculateDeltaCost, bool UpdateOccupancy>
cost_t update_wire_two_bends(const Wire &wire, std::vector<std::vector<int>> &occupancy, const int delta) {
    cost_t delta_cost = 0;
    if (wire.start_x == wire.bend1_x) {
        // Vertical first bend
        for (int y = std::min(wire.start_y, wire.bend1_y); y < std::max(wire.start_y, wire.bend1_y); y++) {
            delta_cost += update_point<CalculateDeltaCost, UpdateOccupancy>(wire.start_x, y, occupancy, delta);
        }

        // Horizontal second bend
        for (int x = std::min(wire.bend1_x, wire.end_x); x < std::max(wire.bend1_x, wire.end_x); x++) {
            delta_cost += update_point<CalculateDeltaCost, UpdateOccupancy>(x, wire.bend1_y, occupancy, delta);
        }

        // Vertical third
        for (int y = std::min(wire.bend1_y, wire.end_y); y <= std::max(wire.bend1_y, wire.end_y); y++) {
            delta_cost += update_point<CalculateDeltaCost, UpdateOccupancy>(wire.bend1_x, y, occupancy, delta);
        }
    } else {
        // Horizontal first bend
        for (int x = std::min(wire.start_x, wire.bend1_x); x < std::max(wire.start_x, wire.bend1_x); x++) {
            delta_cost += update_point<CalculateDeltaCost, UpdateOccupancy>(x, wire.start_y, occupancy, delta);
        }

        // Vertical second bend
        for (int y = std::min(wire.bend1_y, wire.end_y); y < std::max(wire.bend1_y, wire.end_y); y++) {
            delta_cost += update_point<CalculateDeltaCost, UpdateOccupancy>(wire.bend1_x, y, occupancy, delta);
        }

        // Horizontal second
        for (int x = std::min(wire.bend1_x, wire.end_x); x <= std::max(wire.bend1_x, wire.end_x); x++) {
            delta_cost += update_point<CalculateDeltaCost, UpdateOccupancy>(x, wire.bend1_y, occupancy, delta);
        }
    }

    return delta_cost;
}

cost_t calculate_cost(const std::vector<std::vector<int>> &occupancy) {
    int total_cost = 0;
    for (const auto &row: occupancy) {
        for (const int count: row) {
            total_cost += count * count;
        }
    }
    return total_cost;
}

template<bool CalculateDeltaCost, bool UpdateOccupancy>
cost_t update_wire(const Wire &wire, std::vector<std::vector<int>> &occupancy, const int delta) {
    cost_t delta_cost = 0;
    switch (int num_bend = num_bends(wire)) {
        case 0:
            // No bends
            delta_cost = update_wire_no_bend<CalculateDeltaCost, UpdateOccupancy>(wire, occupancy, delta);
            break;
        case 1:
            // One bend
            delta_cost = update_wire_one_bend<CalculateDeltaCost, UpdateOccupancy>(wire, occupancy, delta);
            break;
        case 2:
            // Two bends
            delta_cost = update_wire_two_bends<CalculateDeltaCost, UpdateOccupancy>(wire, occupancy, delta);
            break;
        default:
            std::cerr << "Invalid number of bends: " << num_bend << '\n';
            exit(EXIT_FAILURE);
    }
    return delta_cost;
}

cost_t initialize(const std::vector<Wire> &wires, std::vector<std::vector<int>> &occupancy) {
    /* Initialize occupancy matrix */
    cost_t cost = 0;
    for (const auto &wire: wires) {
        cost += update_wire<true, true>(wire, occupancy, 1);
    }
    std::cout << "Initial cost: " << cost << '\n';
    return cost;
}

void within_wires(std::vector<Wire> &wires, std::vector<std::vector<int>> &occupancy, const int iters) {
    for (int iter = 0; iter < iters; iter++) {
        for (auto &wire: wires) {
            // If the wire is horizontal or vertical, skip
            if (num_bends(wire) == 0) {
                continue;
            }
            // Remove the wire from the occupancy matrix
            cost_t delta_cost = -update_wire<true, true>(wire, occupancy, -1);
            int delta_x = std::abs(wire.start_x - wire.end_x);
            int delta_y = std::abs(wire.start_y - wire.end_y);
            Wire private_wire = wire;

#pragma omp parallel for default(none) shared(wire, delta_cost, delta_x, delta_y, occupancy) firstprivate(private_wire)
            for (int i = 1; i <= delta_x + delta_y; i++) {
                cost_t _delta_cost;
                if (i <= delta_x) {
                    private_wire.bend1_x = std::min(wire.start_x, wire.end_x) + i;
                    private_wire.bend1_y = wire.start_y;
                    _delta_cost = update_wire<true, false>(private_wire, occupancy, 1);
                } else {
                    private_wire.bend1_x = wire.start_x;
                    private_wire.bend1_y = std::min(wire.start_y, wire.end_y) + i - delta_x;
                    _delta_cost = update_wire<true, false>(private_wire, occupancy, 1);
                }
#pragma critical
                {
                    if (_delta_cost < delta_cost) {
                        delta_cost = _delta_cost;
                        wire.bend1_x = private_wire.bend1_x;
                        wire.bend1_y = private_wire.bend1_y;
                    }
                }
            }
            if (does_happen()) {
                random_bend(wire);
            }
            update_wire<false, true>(wire, occupancy, 1);
        }
    }
}
