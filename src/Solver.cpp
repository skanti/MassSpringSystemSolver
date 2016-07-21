#include "Solver.h"
#include <cmath>
#include <iostream>

void Solver::mss_force(double *p_x, double *p_y, double *f_x, double *f_y, int n_nodes,
                       int *a_s, int *b_s, double *k, double *d_eq, int n_springs) {
    for (int i = 0; i < n_springs; i++) {
        //std::cout << a_s[i] << " " << b_s[i] << " " << i << std::endl;
        double dx = p_x[a_s[i]] - p_x[b_s[i]];
        double dy = p_y[a_s[i]] - p_y[b_s[i]];
        double d = std::sqrt(dx * dx + dy * dy);
        double dd = d_eq[i] - d;
        double dxn = dx / d;
        double dyn = dy / d;
        f_x[a_s[i]] += dxn * k[i] * dd;
        f_y[a_s[i]] += dyn * k[i] * dd;
        f_x[b_s[i]] -= dxn * k[i] * dd;
        f_y[b_s[i]] -= dyn * k[i] * dd;
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
