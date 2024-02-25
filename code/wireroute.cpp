/**
 * Parallel VLSI Wire Routing via OpenMP
 * Name 1(andrew_id 1), Name 2(andrew_id 2)
 */

#include "wireroute.h"
#include <random>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <string>
#include <vector>
#include <unistd.h>
#include <omp.h>
#include <limits>

void print_stats(const std::vector<std::vector<int>>& occupancy) {
  int max_occupancy = 0;
  long long total_cost = 0;

  for (const auto& row : occupancy) {
    for (const int count : row) {
      max_occupancy = std::max(max_occupancy, count);
      total_cost += count * count;
    }
  }

  std::cout << "Max occupancy: " << max_occupancy << '\n';
  std::cout << "Total cost: " << total_cost << '\n';
}

void write_output(const std::vector<Wire>& wires, const int num_wires, const std::vector<std::vector<int>>& occupancy, const int dim_x, const int dim_y, const int num_threads, std::string input_filename) {
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
  for (const auto& row : occupancy) {
    for (const int count : row) {
      out_occupancy << count << ' ';
    }
    out_occupancy << '\n';
  }

  out_occupancy.close();

  std::ofstream out_wires(wires_filename, std::fstream:: out);
  if (!out_wires) {
    std::cerr << "Unable to open file: " << wires_filename << '\n';
    exit(EXIT_FAILURE);
  }

  out_wires << dim_x << ' ' << dim_y << '\n' << num_wires << '\n';

  for (const auto& [start_x, start_y, end_x, end_y, bend1_x, bend1_y] : wires) {
    out_wires << start_x << ' ' << start_y << ' ' << bend1_x << ' ' << bend1_y << ' ';

    if (start_y == bend1_y) {
    // first bend was horizontal

      if (end_x != bend1_x) {
        // two bends

        out_wires << bend1_x << ' ' << end_y << ' ';
      }
    } else if (start_x == bend1_x) {
      // first bend was vertical

      if(end_y != bend1_y) {
        // two bends

        out_wires << end_x << ' ' << bend1_y << ' ';
      }
    }
    out_wires << end_x << ' ' << end_y << '\n';
  }

  out_wires.close();
}
void init_wires(std::vector<Wire>& wires,std::vector<std::vector<int>>& occupancy){
	for (auto& wire:wires){

		int deltaX = std::abs(wire.start_x - wire.end_x);
        int deltaY = std::abs(wire.start_y - wire.end_y);

        int x_iterator = (wire.end_x - wire.start_x) > 0 ? 1 : -1;
        int y_iterator = (wire.end_y - wire.start_y) > 0 ? 1 : -1;

		int x = wire.start_x;
        int y = wire.start_y;
		for(int i = 0;i< deltaX;i++){
		    occupancy[x][y]++;
            x +=x_iterator;
		
        }
		if (deltaX!=0 && deltaY!=0){
		    wire.bend1_x = x;
		    wire.bend1_y = y;
		}
		for(int i = 0;i<= deltaY;i++){
		    occupancy[x][y]++;
            y+=y_iterator;
		}
		
	}

}
int select_path(std::vector<int>& costs,double SA_prob){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0,1.0);
    if (dis(gen) > SA_prob){
        auto minIt = std::min_element(costs.begin(),costs.end());
        int mini_index = std::distance(costs.begin(),minIt);
        return mini_index;

    }else{
        std::uniform_int_distribution<> distr(0,costs.size()-1);
        return distr(gen);
    }
    

}
void update_occupmancy(int index,std::vector<std::vector<int>>& occupancy,Wire wire){
    int deltaX = std::abs(wire.start_x-wire.end_x);
    
    int deltaY = std::abs(wire.end_y - wire.start_y);
    int x_iterator = (wire.end_x - wire.start_x) > 0 ? 1 : -1;
    int y_iterator = (wire.end_y - wire.start_y) > 0 ? 1 : -1;
    
    int start_x = wire.start_x;
    int start_y = wire.start_y;
    if (deltaX ==0){
        for(int i = 0;i<deltaY;i++){
            occupancy[start_x][start_y]++;
            start_y+=y_iterator;
        }
        wire.bend1_x = wire.start_x;
        wire.bend1_y =wire.start_y;
        return;
        
    }
    if (deltaY == 0){
        for(int i = 0;i<deltaX;i++){
            occupancy[start_x][start_y]++;
            start_x+=x_iterator;
        }
        wire.bend1_x = wire.start_x;
        wire.bend1_y =wire.start_y;
        return;
    }


    if (index < deltaX){
        int currentIndex = index;

        //  Update occupancy for the horizontal part
        for (int j=0;j<currentIndex;j++){
            occupancy[start_x][start_y]++;
            start_x+=x_iterator;
        }
        wire.bend1_x = start_x;
        wire.bend1_y = start_y;
        // Update occupancy for the vertical part
        for (int j=0;j<deltaY;j++){
            occupancy[start_x][start_y]++;
            start_y+=y_iterator;

        }
        // equal sign
        for (int j=0;j<=(deltaX-currentIndex);j++){
                occupancy[start_x][start_y]++;
                start_x+=x_iterator;
        }

    }else{
        int currentIndex = index;
        for (int j=0;j<currentIndex;j++){
            occupancy[start_x][start_y]++;
            start_y+=y_iterator;
        }
        wire.bend1_x = start_x;
        wire.bend1_y = start_y;
        // Update occupancy for the vertical part
        for (int j=0;j<deltaX;j++){
            occupancy[start_x][start_y]++;
            start_x+=x_iterator;

        }
        // equal sign
        for (int j=0;j<=(deltaY-currentIndex);j++){
            occupancy[start_x][start_y]++;
            start_y+=y_iterator;
        }


    }

}

void within_wires(std::vector<Wire>& wires,std::vector<std::vector<int>>& occupancy,int num_threads,double SA_prob){
	for (auto& wire : wires){
		int deltaX = std::abs(wire.start_x-wire.end_x);
    
    	int deltaY = std::abs(wire.end_y - wire.start_y);

    	int total = deltaX + deltaX;

        int x_iterator = (wire.end_x - wire.start_x) > 0 ? 1 : -1;
        int y_iterator = (wire.end_y - wire.start_y) > 0 ? 1 : -1;

        int x = wire.start_x;
        int y = wire.start_y;

		
		std::vector<int> costs(total);
    	// Clear current path
      	for(int i = 0;i<deltaX;i++){
		  
      		occupancy[x][y]--;
            x+=x_iterator;
		  
      	}
		for(int i = 0;i<=deltaY;i++){
		  
      		occupancy[x][y]--;
            y-=y_iterator;

		}
    // start process
		int i;

    	#pragma omp parallel for private(i)
    	for(i = 0;i<total;i++){
			if (i < deltaX){
                int currentIndex = i;
        		int cost = std::numeric_limits<int>::max();
                int start_x = wire.start_x;
                int start_y = wire.start_y;

                //  Update occupancy for the horizontal part
                for (int j=0;j<currentIndex;j++){
                    int current_occupancy = occupancy[start_x][start_y]+1;
                    cost+=current_occupancy * current_occupancy;
                    start_x+=x_iterator;
                }
                 // Update occupancy for the vertical part
                for (int j=0;j<deltaY;j++){
                    int current_occupancy = occupancy[start_x][start_y]+1;
                    cost+=current_occupancy * current_occupancy;
                    start_y+=y_iterator;

                }
                // equal sign
                for (int j=0;j<=(deltaX - currentIndex);j++){
                    int current_occupancy = occupancy[start_x][start_y]+1;
                    cost+=current_occupancy * current_occupancy;
                    start_x+=x_iterator;
                }
                // assign cost
                costs[i] = cost;
        }else{
            int currentIndex = i-deltaX;
            int cost = std::numeric_limits<int>::max();
            int start_x = wire.start_x;
            int start_y = wire.start_y;

            for (int j=0;j<currentIndex;j++){
                int current_occupancy = occupancy[start_x][start_y]+1;
                cost+=current_occupancy * current_occupancy;
                start_y+=y_iterator;
            }
            for (int j=0;j<deltaX;j++){
                int current_occupancy = occupancy[start_x][start_y]+1;
                cost+=current_occupancy * current_occupancy;
                start_x+=x_iterator;
            }
            // equal sign
            for (int j=0;j<=(deltaY - currentIndex);j++){
                int current_occupancy = occupancy[start_x][start_y]+1;
                cost+=current_occupancy * current_occupancy;
                start_y+=y_iterator;
            }
            costs[i] = cost;
            

        }



    	}
        int path_index = select_path(costs,SA_prob);
        update_occupmancy(path_index,occupancy,wire);
        

        
    
    
    
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
        std::cerr << "Usage: " << argv[0] << " -f input_filename -n num_threads [-p SA_prob] [-i SA_iters] -m parallel_mode -b batch_size\n";
        exit(EXIT_FAILURE);
    }
  }

  // Check if required options are provided
  if (empty(input_filename) || num_threads <= 0 || SA_iters <= 0 || (parallel_mode != 'A' && parallel_mode != 'W') || batch_size <= 0) {
    std::cerr << "Usage: " << argv[0] << " -f input_filename -n num_threads [-p SA_prob] [-i SA_iters] -m parallel_mode -b batch_size\n";
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

  for (auto& wire : wires) {
    fin >> wire.start_x >> wire.start_y >> wire.end_x >> wire.end_y;
    wire.bend1_x = wire.start_x;
    wire.bend1_y = wire.start_y;
  }

  /* Initialize any additional data structures needed in the algorithm */
  init_wires(wires,occupancy);

  const double init_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - init_start).count();
  std::cout << "Initialization time (sec): " << std::fixed << std::setprecision(10) << init_time << '\n';

  const auto compute_start = std::chrono::steady_clock::now();
  if (parallel_mode == 'W') {
	for(int i=0;i<SA_iters;i++){
		within_wires(wires,occupancy,num_threads,SA_prob);
	}
  }else{
	for(int i=0;i<SA_iters;i++){
		within_wires(wires,occupancy,num_threads,SA_prob);
	}
    

  }

  /** 
   * Implement the wire routing algorithm here
   * Feel free to structure the algorithm into different functions
   * Don't use global variables.
   * Use OpenMP to parallelize the algorithm. 
   */
  
   

  const double compute_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - compute_start).count();
  std::cout << "Computation time (sec): " << compute_time << '\n';

  /* Write wires and occupancy matrix to files */

  print_stats(occupancy);
  write_output(wires, num_wires, occupancy, dim_x, dim_y, num_threads, input_filename);
}

validate_wire_t Wire::to_validate_format(void) const {
  /* TODO(student): Implement this if you want to use the wr_checker. */
  /* See wireroute.h for details on validate_wire_t. */
  throw std::logic_error("to_validate_format not implemented.");
}
