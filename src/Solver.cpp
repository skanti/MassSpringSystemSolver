#include "Solver.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>

void Solver::mss_force(double *p_x, double *p_y, double *f_x, double *f_y, int *index, int n_nodes, int *a_s, int *b_s,
                       double *k, double *d_eq, int n_springs) {
    for (int i = 0; i < n_springs; i++) {
        int asi = index[a_s[i]];
        int bsi = index[b_s[i]];
        double dx = p_x[asi] - p_x[bsi];
        double dy = p_y[asi] - p_y[bsi];
        double d = std::sqrt(dx * dx + dy * dy);
        double dd = d_eq[i] - d;
        double dxn = dx / d;
        double dyn = dy / d;
        f_x[asi] += dxn * k[i] * dd;
        f_y[asi] += dyn * k[i] * dd;
        f_x[bsi] -= dxn * k[i] * dd;
        f_y[bsi] -= dyn * k[i] * dd;
    }
}

void Solver::external_force(double *p_x, double *p_y, double *v_x, double *v_y, double *f_x, double *f_y, double *m,
                            int n_nodes) {
    for (int i = 0; i < n_nodes; i++) {
        f_y[i] += -9.81;
    }
}

void Solver::euler_forward(double *p_x, double *p_y, double *v_x, double *v_y, double *f_x, double *f_y, double *m,
                           double dt, int n_nodes) {
    for (int i = 0; i < n_nodes; i++) {
        v_x[i] = v_x[i] * (1.0 - dt) + f_x[i] * dt / m[i];
        v_y[i] = v_y[i] * (1.0 - dt) + f_y[i] * dt / m[i];
        p_x[i] += v_x[i] * dt;
        p_y[i] += v_y[i] * dt;
    }
}

int Solver::find_pair_duplicate(int ai, int bi, int *a, int *b, int n_springs) {
    int j_duplicate = -1;
    for (int j = 0; j < n_springs; j++) {
        j_duplicate += (ai == a[j]) && (bi == b[j]) * (j + 1);
    }
    return j_duplicate;
}
