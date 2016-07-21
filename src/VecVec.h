//
// Created by amon on 7/21/16.
//

#ifndef SPRINGS2D_VECVEC_H
#define SPRINGS2D_VECVEC_H


struct VecVec {
    VecVec(int n_rows_, int n_cols_) : n_rows(n_rows_), n_cols(n_cols_), data(n_rows * n_cols) {}

    double &operator()(int i_row, int j_col) {
        return data[i_row * n_cols + j_col];
    }

    double *operator()(int i_row) {
        return &data[i_row * n_cols];
    }

    void resize(int n_rows_, int n_cols_) {
        n_rows = n_rows_;
        n_cols = n_cols_;
        data.resize(n_rows * n_cols);
    }

    int n_rows;
    int n_cols;
    std::vector<double> data;
};

#endif //SPRINGS2D_VECVEC_H
