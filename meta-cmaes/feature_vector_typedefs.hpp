
#ifndef FEATURE_VEC_TYPES_HPP
#define FEATURE_VEC_TYPES_HPP




#define EIGEN_DENSEBASE_PLUGIN "EigenDenseBaseAddons.h"
#include <Eigen/Dense>



const int NUM_BASE_FEATURES = 14; // number of base features
// const int NUM_TOP_CELLS = 15;      // number of cells in the meta-map
const int NUM_BOTTOM_FEATURES = 2; // number of features for bottom level maps
const int NUM_GENES = NUM_BASE_FEATURES * NUM_BOTTOM_FEATURES;

/* base-features */
typedef Eigen::Matrix<float, NUM_BASE_FEATURES, 1, Eigen::DontAlign, NUM_BASE_FEATURES, 1> base_features_t; // 0 options and size cannot grow

/* weights to construct bottom-level map features from base_features */
typedef Eigen::Matrix<float, NUM_BOTTOM_FEATURES, NUM_BASE_FEATURES, Eigen::DontAlign, NUM_BOTTOM_FEATURES, NUM_BASE_FEATURES> weight_t;

/* bottom-level map features */
typedef Eigen::Matrix<float, NUM_BOTTOM_FEATURES, 1, Eigen::DontAlign, NUM_BOTTOM_FEATURES, 1> bottom_features_t;



#endif
