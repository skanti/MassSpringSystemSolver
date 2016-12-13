#pragma once

#include <sstream>
#include "Nodes.h"
#include "Springs.h"
#include <unordered_map>
#include <bitset>
#include <utility>
#include <algorithm>
#include <numeric>

template<typename value_type_spring>
void parse_csr(std::vector<std::pair<std::pair<int32_t, int32_t>, value_type_spring>> &coo, CSR<value_type_spring> &matrix) {
	int n_nz = coo.size();
	matrix.i_y[0] = coo[0].first.first;
	matrix.i_x[0] = coo[0].first.second;
	matrix.data[0] = coo[0].second;
	int i1 = 1; // <-- current row pointer
	matrix.i_y[1] = 1;
	for (int i = 1; i < n_nz; i++) {
		matrix.i_x[i] = coo[i].first.second;
		matrix.data[i] = coo[i].second;

		if (i1 == coo[i].first.first) {
			i1++;
			matrix.i_y[i1] += matrix.i_y[i1-1] + 1; 
		} else { 
			matrix.i_y[i1]++;
			matrix.i_y[i1 + 1] = 0;
		}
	}
};

template<typename value_type_node, typename value_type_spring>
static void load_mesh_ply2(std::string file, Nodes<value_type_node> &nodes, Springs<value_type_spring> &springs) {
	
	// -> read header
    int n_nodes, n_triangles;
    std::ifstream infile(file);
    if (!infile.is_open()) throw std::runtime_error("Cannot open file.");

    std::string line;
    std::getline(infile, line);
    std::stringstream(line) >> n_nodes;
    std::getline(infile, line);
    std::stringstream(line) >> n_triangles;
    std::cout << "n_nodes: " << n_nodes << " n_triangles: " << n_triangles << std::endl;
    // <-

    // -> reserve node memory
    nodes.init(n_nodes);
    for (int i = 0; i < n_nodes; i++) {
    	value_type_node px, py, pz;
    	std::getline(infile, line);
    	std::stringstream(line) >> px >> py >> pz;
    	nodes.set(i, px, py, 0, 0, 1.0);
    }
    // <-

    // -> find number of springs
    std::unordered_map<int64_t, int> hashmap;
    auto insert_unique = [&hashmap](int32_t a, int32_t b) {
    	int64_t key = ((int64_t)std::min(a, b) << 32) + (int64_t)std::max(a, b); if (hashmap.find(key) == hashmap.end()) hashmap.insert({key, 1});
    };
    for (int i = 0; i < n_triangles; i++) {
    	int32_t type_polygon, a, b, c;
    	std::getline(infile, line);
    	std::stringstream(line) >> type_polygon >> a >> b >> c;
    	insert_unique(a, b);
    	insert_unique(a, c);
    	insert_unique(b, c);
    }
    // <-

    // -> create spring sparse matrices
    int n_springs = hashmap.size();
    springs.init(n_nodes, n_springs);
    std::vector<std::pair<std::pair<int32_t, int32_t>, value_type_spring>> coo;
    int i = 0;
    for (auto itr = hashmap.begin(); itr != hashmap.end(); itr++, i++) {
    	int64_t key = itr->first;
    	int32_t a = key >> 32;
    	int32_t b = key >> 0;
    	coo.push_back({{a, i}, -1});
    	coo.push_back({{b, i},  1});
    }
    std::sort(coo.begin(), coo.end());
    parse_csr(coo, springs.A);
    // <-

   exit(0);
}
