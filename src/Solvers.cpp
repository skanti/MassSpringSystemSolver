#include "Solvers.h"
#include <algorithm>
#include <numeric>

void Solvers::sort(int *a, int *b, int *key, int n_size) {
    std::iota(&key[0], &key[n_size - 1], 0);
    auto compare = [&](const int &i, const int &j) {
        return (a[i] < a[j])
               || ((a[i] == a[j]) && (b[i] <= b[j]));
    };
    std::sort(&key[0], &key[n_size - 1], compare);
}