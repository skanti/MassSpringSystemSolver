#pragma once

#include <vector>
#include <utility>
#include <unordered_map>

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


    void set(int i, int ai, int bi, double ki, double deqi) {
        a[i] = ai;
        b[i] = bi;
        k[i] = ki;
        d_eq[i] = deqi;
    }

    void push_back(int ai, int bi, double ki, double deqi) {
        set(n_size, ai, bi, ki, deqi);
        n_size++;
    }

    void push_back_sym_if_unique(int ai, int bi, double ki, double deqi) {
        int abi = ai <= bi ? (ai << 16) + bi : (bi << 16) + ai;
        if (hashkey.insert({abi, n_size}).second) {
            set(n_size, ai, bi, ki, deqi);
            n_size++;
        }
    }

    void remove_sym_if_unique(int ai, int bi) {
        int hki = ai <= bi ? (ai << 16) + bi : (bi << 16) + ai;
        if (hashkey.find(hki) != hashkey.end()) {
            remove(hashkey[hki]);
            hashkey.erase(hki);
        }
    }


    void swap(int i, int j) {
        std::swap(a[i], a[j]);
        std::swap(b[i], b[j]);
        std::swap(k[i], k[j]);
        std::swap(d_eq[i], d_eq[j]);
        int64_t hki = ((int64_t)a[i] << 32) + b[i];
        int64_t hkj = ((int64_t)a[j] << 32) + b[j];
        std::swap(hashkey[hki], hashkey[hkj]);

    }

    static void
    set_deq_by_given_state(double *p_x, double *p_y, int *index, int *a, int *b, double *deq, int n_springs) {
        for (int i = 0; i < n_springs; i++) {
            int ai = index[a[i]];
            int bi = index[b[i]];
            double dx = p_x[ai] - p_x[bi];
            double dy = p_y[ai] - p_y[bi];
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
    std::unordered_map<int64_t, int> hashkey;
    std::vector<int, AlignedAllocator<int, 32>> a;
    std::vector<int, AlignedAllocator<int, 32>> b;
    std::vector<double, AlignedAllocator<double, 32>> k;
    std::vector<double, AlignedAllocator<double, 32>> d_eq;
    std::vector<int, AlignedAllocator<int, 32>> key;
    // <-

    // -> temporary/dynamic
    std::vector<int, AlignedAllocator<int, 32>> tmp;
    // <-
};

