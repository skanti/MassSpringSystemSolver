#pragma once

#include <sstream>
#include <unordered_map>
#include <utility>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <mkl.h>
#include "Nodes.h"
#include "Springs.h"
#include "Common.h"


template<typename value_type_nodes, typename value_type_springs>
static void load_mesh_ply2(std::string file, Nodes<value_type_nodes> &nodes, Springs<value_type_springs> &springs, SparseMatrix<value_type_springs> &A) {
	
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
    	value_type_nodes px, py, pz;
    	std::getline(infile, line);
    	std::stringstream(line) >> px >> py >> pz;
    	nodes.set(i, px, py, pz, 0, 0, 0, 1.0);
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
    springs.init(n_springs);
    typedef Eigen::Triplet<value_type_nodes> Triplet;
    std::vector<Triplet> coo_A, coo_K;
    int i = 0;
    for (auto itr = hashmap.begin(); itr != hashmap.end(); itr++, i++) {
    	int64_t key = itr->first;
    	int32_t a = key >> 32;
    	int32_t b = key >> 0;
        value_type_springs k = 10.0;
        springs.k[i] = k;
        coo_K.push_back(Triplet(i, i, k));
    	coo_A.push_back(Triplet(a, i, -1));
        coo_A.push_back(Triplet(b, i,  1));
    }
    A.setFromTriplets(coo_A.begin(), coo_A.end());
    // <-
}


template<typename value_type_nodes, typename value_type_springs>
static void load_mesh_ply(std::string file, Nodes<value_type_nodes> &nodes, Springs<value_type_springs> &springs, SparseMatrix<value_type_springs> &A) {
    
    // -> read header
    int n_nodes, n_triangles;
    std::ifstream infile(file);
    if (!infile.is_open()) throw std::runtime_error("Cannot open file.");

    std::string line, dummy;
    std::getline(infile, line); std::getline(infile, line); 
     std::getline(infile, line);
    std::stringstream(line) >> dummy >> dummy >> n_nodes;
    while(std::getline(infile, line)) {
        if (line.find("element") == 0) {
            std::stringstream(line) >> dummy >> dummy >> n_triangles;
            break;
        }
    }
    //std::stringstream(line) >> n_triangles;
    std::cout << "n_nodes: " << n_nodes << " n_triangles: " << n_triangles << std::endl;
    // <-

    // -> skip 'property_list ...' and 'end_header'
    std::getline(infile, line); std::getline(infile, line); 
    // <-

    // -> reserve node memory
    nodes.init(n_nodes);
    for (int i = 0; i < n_nodes; i++) {
        value_type_nodes px, py, pz;
        std::getline(infile, line);
        std::stringstream(line) >> px >> py >> pz;
        nodes.set(i, px, py, pz, 0, 0, 0, 1.0);
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
    springs.init(n_springs);
    typedef Eigen::Triplet<value_type_nodes> Triplet;
    std::vector<Triplet> coo_A, coo_K;
    int i = 0;
    for (auto itr = hashmap.begin(); itr != hashmap.end(); itr++, i++) {
        int64_t key = itr->first;
        int32_t a = key >> 32;
        int32_t b = key >> 0;
        value_type_springs k = 10.0;
        springs.k[i] = k;
        coo_K.push_back(Triplet(i, i, k));
        coo_A.push_back(Triplet(a, i, -1));
        coo_A.push_back(Triplet(b, i,  1));
    }
    springs.A.setFromTriplets(coo_A.begin(), coo_A.end());
    // <-
}


template<typename value_type_nodes>
void normalize_and_recenter_nodes( Nodes<value_type_nodes> &nodes) {
    // -> find min max alue
    auto ix_minmax  = std::minmax_element(&nodes.px[0], &nodes.px[0] + nodes.n_size);
    auto iy_minmax  = std::minmax_element(&nodes.py[0], &nodes.py[0] + nodes.n_size);
    auto iz_minmax  = std::minmax_element(&nodes.pz[0], &nodes.pz[0] + nodes.n_size);
    // <-

    // -> find abs maximum
    value_type_nodes px_absmax = std::max(std::abs(ix_minmax.first[0]), std::abs(ix_minmax.second[0]));
    value_type_nodes py_absmax = std::max(std::abs(iy_minmax.first[0]), std::abs(iy_minmax.second[0]));
    value_type_nodes pz_absmax = std::max(std::abs(iz_minmax.first[0]), std::abs(iz_minmax.second[0]));
    value_type_nodes p_max = std::max(px_absmax,  std::max(py_absmax, pz_absmax));
    // <-

    // -> normalize
    value_type_nodes com_x = 0, com_y = 0, com_z = 0;
    for (int i = 0; i < nodes.n_size; i++) {
        nodes.px[i] /= p_max;
        com_x += nodes.px[i];
        nodes.py[i] /= p_max;
        com_y += nodes.py[i];
        nodes.pz[i] /= p_max;
        com_z += nodes.pz[i];
    }
    com_x /= (value_type_nodes) nodes.n_size;
    com_y /= (value_type_nodes) nodes.n_size;
    com_z /= (value_type_nodes) nodes.n_size;
    // <- 

    // -> shift
    for (int i = 0; i < nodes.n_size; i++) {
        nodes.px[i] -= com_x;
        nodes.py[i] -= com_y;
        nodes.pz[i] -= com_z;
    }
    // <-
}


template<typename value_type_nodes, typename value_type_springs>
void create_cloth( Nodes<value_type_nodes> &nodes,  Springs<value_type_springs> &springs, SparseMatrix<value_type_springs> &A, int n) {
    // -> create nodes
    int n_nodes = n*n;
    nodes.init(n_nodes);
    for (int i = 0; i < n; i++){
        for (int j = 0; j < n; j++){
            value_type_nodes px = (value_type_nodes)j/(value_type_nodes)n;
            value_type_nodes py = 1.0 - (value_type_nodes)i/(value_type_nodes)n;
            value_type_nodes pz = 0;
            nodes.set(i*n + j, px, py, pz, 0, 0, 0, 1.0);
        }
    }
    // <-

    // -> create springs
    int n_springs = 3*(n-1)*(n-1) + 2*(n-1);
    typedef Eigen::Triplet<value_type_nodes> Triplet;
    std::vector<Triplet> coo_A;
    int i_counter = 0;
    for (int i = 0; i < n; i++){
        for (int j = 0; j < n; j++){
            int k = i*n + j;
            value_type_nodes a = i*n + (j + 1);
            value_type_nodes b = (i + 1)*n + j;
            value_type_nodes c = (i + 1)*n + (j + 1);
            if (j + 1 < n) {
                coo_A.push_back(Triplet(k, i_counter, -1));
                coo_A.push_back(Triplet(a, i_counter, 1));
                i_counter++;
            } 
            if (i + 1 < n) {          
                coo_A.push_back(Triplet(k, i_counter, -1));
                coo_A.push_back(Triplet(b, i_counter, 1));
                i_counter++;
            }
            if (i + 1 < n && j + 1 < n) {
                coo_A.push_back(Triplet(k, i_counter, -1));
                coo_A.push_back(Triplet(c, i_counter, 1));
                i_counter++;
            }
        }
    }
    assert(n_springs == i_counter);
    springs.init(n_springs);
    std::iota(&springs.key[0], &springs.key[0] + springs.n_size_reserve, 0);
    A.setFromTriplets(coo_A.begin(), coo_A.end());
    // <-
}

template<typename value_type_nodes, typename value_type_springs>
void create_microtubule( Nodes<value_type_nodes> &nodes,  Springs<value_type_springs> &springs, 
    SparseMatrix<value_type_springs> &A, Vector<int> &T0, Vector<int> &T1, const int n) {
    
    // -> create nodes
    int n_nodes = 13*2*n;
    nodes.init(n_nodes);
    for (int i = 0; i < 13; i++){
        for (int j = 0; j < n*2; j++){
            value_type_nodes px = std::cos(M_PI*i/13.0f*2.0);
            value_type_nodes py = i/12.0 + j/2.5;
            value_type_nodes pz = std::sin(M_PI*i/13.0f*2.0);
            nodes.set(i*2*n + j, px, py, pz, 0, 0, 0, 1.0);
        }
        T0[i] = n*2; 
    }
    // <-

    // -> create springs
    typedef Eigen::Triplet<value_type_nodes> Triplet;
    std::vector<Triplet> coo_A;
    int i_counter = 0;

       for (int i = 0; i < 13; i++){
        for (int j = 0; j < n*2 - 1; j++){
            value_type_nodes a = i*2*n + j;
            value_type_nodes b = i*2*n + j + 1;
            coo_A.push_back(Triplet(a, i_counter, -1));
            coo_A.push_back(Triplet(b, i_counter, 1));
            i_counter++;
        }
        T1[i] = i_counter;
    }

    for (int k = 0; k < 3; k++) {
        for (int j = k; j < n*2; j += 3) {
            for (int i = 0; i < 13; i++){
                value_type_nodes a = i*2*n + j;
                value_type_nodes b = ((i + 1)%13)*2*n + j + (i == 12)*3;
                if (b < n_nodes && j + (i == 12)*3 < n*2) {
                    coo_A.push_back(Triplet(a, i_counter, -1));
                    coo_A.push_back(Triplet(b, i_counter, 1));
                    i_counter++;
                }
            }
        }
        T1[13 + k] = i_counter;
    }
    springs.init(i_counter);
    std::iota(&springs.key[0], &springs.key[0] + springs.n_size_reserve, 0);
    A.setFromTriplets(coo_A.begin(), coo_A.end());
    // <-
}