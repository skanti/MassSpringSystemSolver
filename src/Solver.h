#ifndef SPRINGS2D_SOLVERS_H
#define SPRINGS2D_SOLVERS_H

#include <vector>

#include <vector>

class Solver {
public:
    static void mss_force(double *p_x, double *p_y, double *f_x, double *f_y, int *index, int n_nodes, int *a, int *b,
                              double *k, double *d_eq, int n_springs);


    static void
    external_force(double *p_x, double *p_y, double *v_x, double *v_y, double *f_x, double *f_y, double *m,
                   int n_nodes);


    static void
    euler_forward(double *p_x, double *p_y, double *v_x, double *v_y, double *f_x, double *f_y, double *m,
                  double dt, int n_nodes);

    static void 
    kavan(double *p_x, double *p_y, double *v_x, double *v_y, double *f_x, double *f_y, double *m,
                  double dt, int n_nodes);

    static int find_pair_duplicate(int ai, int bi, int *a, int *b, int n_springs);
};


#endif //SPRINGS2D_SOLVERS_H
