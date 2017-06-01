#pragma once

#include <sstream>
#include <unordered_map>
#include <utility>
#include <algorithm>
#include <numeric>
#include <cmath>
#include "Nodes.h"
#include "Springs.h"
#include "Common.h"



template<typename value_type_nodes>
void normalize_and_recenter_nodes( Nodes<value_type_nodes> &nodes) {
    // -> find min max alue
    auto i_minmax  = std::minmax_element(&nodes.p(0, 0), &nodes.p(0, 0) + 3*nodes.n_size);
    // <-

    // -> find abs maximum
    value_type_nodes p_absmax = std::max(std::abs(i_minmax.first[0]), std::abs(i_minmax.second[0]));
    // <-

    // -> normalize
    Vector3<value_type_nodes> com;
    for (int i = 0; i < nodes.n_size; i++) {
        nodes.p.col(i) /= p_absmax;
        com += nodes.p.col(i);
    }
    com /= (value_type_nodes) nodes.n_size;
    // <- 

    // -> shift
    for (int i = 0; i < nodes.n_size; i++) {
        nodes.p.col(i) -= com;
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
    SparseMatrix<value_type_springs> &A, Vector<int> &T0, Vector<int> &T1, Vector<int> &S0, Vector<int> &V0, 
    std::unordered_map<int64_t, unsigned char> &L, 
    const int n_pt, const int n_pitch, const int n) {
    
    // -> create nodes
    int n_nodes = n_pt*2*n;
    nodes.init(n_nodes);
    for (int i = 0; i < n_pt; i++){
        for (int j = 0; j < n*2; j++){
            int k = i*2*n + j;
            value_type_nodes px = std::cos(M_PI*i/(value_type_nodes)n_pt*2.0);
            value_type_nodes py = i/((value_type_nodes)n_pt - 1.0) + j/(n_pitch*0.75);
            value_type_nodes pz = std::sin(M_PI*i/(value_type_nodes)n_pt*2.0);
            nodes.set(k, px, py, pz, 0, 0, 0, 1.0);
            S0[k] = k;
            V0[k] = i;
        }
        T0[i] = n*2; 
    }
    // <-

    // -> create springs
    typedef Eigen::Triplet<value_type_nodes> Triplet;
    std::vector<Triplet> coo_A;
    int i_counter = 0;

       for (int i = 0; i < n_pt; i++){
        for (int j = 0; j < n*2 - 1; j++){
            value_type_nodes a = i*2*n + j;
            value_type_nodes b = i*2*n + j + 1;
            int64_t ab = a <= b ? ((int64_t)a << 32) + b : ((int64_t)b << 32) + a;
            L.insert({ab, (unsigned char)1});
            coo_A.push_back(Triplet(a, i_counter, -1));
            coo_A.push_back(Triplet(b, i_counter, 1));
            i_counter++;
        }
        T1[i] = i_counter;
    }

    for (int k = 0; k < n_pitch; k++) {
        for (int j = k; j < n*2; j += n_pitch) {
            for (int i = 0; i < n_pt; i++){
                value_type_nodes a = i*2*n + j;
                value_type_nodes b = ((i + 1)%n_pt)*2*n + j + (i == n_pt - 1)*n_pitch;
                int64_t ab = a <= b ? ((int64_t)a << 32) + b : ((int64_t)b << 32) + a;
                L.insert({ab, (unsigned char)1});
                if (b < n_nodes && j + (i == n_pt - 1)*n_pitch < n*2) {
                    coo_A.push_back(Triplet(a, i_counter, -1));
                    coo_A.push_back(Triplet(b, i_counter, 1));
                    i_counter++;
                }
            }
        }
        T1[n_pt + k] = i_counter;
    }
    springs.init(i_counter);
    std::iota(&springs.key[0], &springs.key[0] + springs.n_size_reserve, 0);
    A.setFromTriplets(coo_A.begin(), coo_A.end());
    // <-
}