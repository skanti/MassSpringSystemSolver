#ifndef SPRINGS_H
#define SPRINGS_H

#include <vector>
#include <utility>


struct Springs {
    Springs(int n_size_reserved_) :
            n_size(0),
            n_size_reserved(n_size_reserved_),
            a(n_size_reserved), b(n_size_reserved),
            k(n_size_reserved), d_eq(n_size_reserved),
            key(n_size_reserved), tmp(n_size_reserved) {};

    void reserve(int n_size_reserved_) {
        n_size_reserved = n_size_reserved_;
        a.resize(n_size_reserved);
        b.resize(n_size_reserved);
        k.resize(n_size_reserved);
        d_eq.resize(n_size_reserved);
        key.resize(n_size_reserved);
        tmp.resize(n_size_reserved);
    }


    void set(int i, int ai, int bi, double ki, double deqi, int keyi) {
        a[i] = ai;
        b[i] = bi;
        k[i] = ki;
        d_eq[i] = deqi;
        key[i] = keyi;
    }

    void push_back(int ai, int bi, double ki, double deqi, int keyi) {
        set(n_size, ai, bi, ki, deqi, keyi);
        n_size++;
    }


    void swap(int i, int j) {
        std::swap(a[i], a[j]);
        std::swap(b[i], b[j]);
        std::swap(k[i], k[j]);
        std::swap(d_eq[i], d_eq[j]);
        std::swap(key[i], key[j]);
    }

    static void set_deq_by_given_state(double *p_x, double *p_y, int *a, int *b, double *deq, int n_springs) {
        for (int i = 0; i < n_springs; i++) {
            double dx = p_x[a[i]] - p_x[b[i]];
            double dy = p_y[a[i]] - p_y[b[i]];
            deq[i] = std::sqrt(dx * dx + dy * dy);
        };
    }


    void remove(int i) {
        swap(i, n_size - 1);
        n_size--;
    }


    // -> springs
    int n_size;
    int n_size_reserved;
    std::vector<int> a;
    std::vector<int> b;
    std::vector<double> k;
    std::vector<double> d_eq;
    std::vector<int> key;
    // <-

    // -> temporary/dynamic
    std::vector<int> tmp;
    // <-
};

#endif //GRAVITYASSIST_PHYSICALPROPERTIES_H
