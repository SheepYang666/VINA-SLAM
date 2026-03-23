#pragma once

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <vector>

#define HASH_P 1000033
#define MAX_N 100000000000
#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0
#define PLM(a) vector<Eigen::Matrix<double, a, a>, Eigen::aligned_allocator<Eigen::Matrix<double, a, a>>>
#define PLV(a) vector<Eigen::Matrix<double, a, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, a, 1>>>

#define G_m_s2 9.8
#define DIM 15
#define NMATCH 5

// Terminal colors
#define RESET "\033[0m"
#define BLACK "\033[30m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN "\033[36m"
#define WHITE "\033[37m"

#define BOLDRED "\033[1;31m"
#define BOLDGREEN "\033[1;32m"
#define BOLDYELLOW "\033[1;33m"
#define BOLDBLUE "\033[1;34m"
#define BOLDCYAN "\033[1;36m"

#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * M_PI / 180.0)
#endif

typedef pcl::PointXYZINormal PointType;
