#pragma once

#include <sstream>
#include "Nodes.h"
#include "Springs.h"
#include <unordered_map>
#include <utility>
#include <algorithm>
#include <numeric>
#include <mkl.h>
#include "Common.h"


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
    typedef Eigen::Triplet<value_type_node> Triplet;
    std::vector<Triplet> coo_A, coo_K;
    int i = 0;
    for (auto itr = hashmap.begin(); itr != hashmap.end(); itr++, i++) {
    	int64_t key = itr->first;
    	int32_t a = key >> 32;
    	int32_t b = key >> 0;
        value_type_spring k = 10.0;
        springs.k[i] = k;
        coo_K.push_back(Triplet(i, i, k));
    	coo_A.push_back(Triplet(a, i, -1));
        coo_A.push_back(Triplet(b, i,  1));
    }
    springs.A.setFromTriplets(coo_A.begin(), coo_A.end());
    springs.K.setFromTriplets(coo_K.begin(), coo_K.end());
    // <-
}
