#pragma once

#include <cstdlib>
#include <cassert>
#include <eigen3/Eigen/Sparse>

template <typename T>
using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;

template <typename T>
using SparseMatrix = Eigen::SparseMatrix<T>;