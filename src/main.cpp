#include <iostream>
#include "MSNWorld.h"
#include <string>

std::unique_ptr<MSNWorld> MSNWorld::msn2d_world = nullptr;

int main(int argc, char *argv[]) {
    assert(argc == 4 && "Paramters: optimization_method n_iteration_optimization sigma_langevin");

    std::string optimization_method = std::string(argv[1]);
    int n_iteration_optimization = std::atoi(argv[2]);
    double sigma_langevin = std::atof(argv[3]);

    std::cout << "Job. method: " << optimization_method << " n_itr: " << n_iteration_optimization << " sigma_langevin: " << sigma_langevin << std::endl;

    MSNWorld::init();
    for (std::size_t j = 0; j < 40*60; j++) {
        // std::cout << (j + 1) <<  "\r";
        std::cout << (j + 1) <<  std::endl;
    	MSNWorld::get_instance().advance(j, optimization_method, n_iteration_optimization, sigma_langevin);
        // fflush(stdout);
    }
    std::cout << std::endl;
    
    return 0;
}
