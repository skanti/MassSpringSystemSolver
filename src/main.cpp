#include <iostream>
#include "MSNWorld.h"
#include <string>

std::unique_ptr<MSNWorld> MSNWorld::msn2d_world = nullptr;

int main(int argc, char *argv[]) {
    assert(argc == 5 && "Paramters: optimization_method n_iteration_optimization sigma_langevin i_run");

    std::string optimization_method = std::string(argv[1]);
    int n_iteration_optimization = std::atoi(argv[2]);
    double sigma_langevin = std::atof(argv[3]);
	int i_run = std::atoi(argv[4]);

    std::cout << "***Job*** method: " << optimization_method << " n_itr: " << n_iteration_optimization << " sigma_langevin: " << sigma_langevin << " i_run: " << i_run << std::endl;

    MSNWorld::init();
    MSNWorld::get_instance().init_filename_and_seed_and_method(optimization_method, n_iteration_optimization, sigma_langevin, i_run);
	for (std::size_t j = 0; j < 40*60; j++) {
    	MSNWorld::get_instance().advance(j, n_iteration_optimization, sigma_langevin);
    }
    
	return 0;
}
