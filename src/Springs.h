#ifndef SPRINGS_H
#define SPRINGS_H

#include <vector>
#include <utility>


struct Springs {
    Springs(int n_size_reserved_) :
            n_size(0),
            n_size_reserved(n_size_reserved_),
            a(n_size_reserved), b(n_size_reserved),
            k(n_size_reserved), key(n_size_reserved), tmp(n_size_reserved) {};

    void reserve(int n_size_reserved_) {
        n_size_reserved = n_size_reserved_;
        a.resize(n_size_reserved);
        b.resize(n_size_reserved);
        k.resize(n_size_reserved);
        key.resize(n_size_reserved);
        tmp.resize(n_size_reserved);
    }


    void set(int i, int ai, int bi, double ki, int keyi) {
        a[i] = ai;
        b[i] = bi;
        k[i] = ki;
        key[i] = keyi;
    }

    void push_back(int ai, int bi, double ki, int keyi) {
        set(n_size, ai, bi, ki, keyi);
        n_size++;
    }


    void swap(int i, int j) {
        std::swap(a[i], a[j]);
        std::swap(b[i], b[j]);
        std::swap(k[i], k[j]);
        std::swap(key[i], key[j]);
    }


    void remove_spring(int i) {
        swap(i, n_size - 1);
        n_size--;
    }


    // -> springs
    int n_size;
    int n_size_reserved;
    std::vector<int> a;
    std::vector<int> b;
    std::vector<double> k;
    std::vector<int> key;
    // <-

    // -> temporary/dynamic
    std::vector<int> tmp;
    // <-
};

#endif //GRAVITYASSIST_PHYSICALPROPERTIES_H
