
#ifndef FEATURE_VEC_TYPES_HPP
#define FEATURE_VEC_TYPES_HPP


#define LINEAR 0
#define SELECTION 1
#define NONLINEAR 2

#define EIGEN_DENSEBASE_PLUGIN "EigenDenseBaseAddons.h"
#include <Eigen/Dense>



const int NUM_BASE_FEATURES = 14; // number of base features
// const int NUM_TOP_CELLS = 15;      // number of cells in the meta-map
const int NUM_BOTTOM_FEATURES = 4; // number of features for bottom level maps

/* base-features */
typedef Eigen::Matrix<float, NUM_BASE_FEATURES, 1, Eigen::DontAlign, NUM_BASE_FEATURES, 1> base_features_t; // 0 options and size cannot grow
typedef Eigen::Matrix<size_t, NUM_BOTTOM_FEATURES, 1, Eigen::DontAlign, NUM_BOTTOM_FEATURES, 1> bottom_indices_t; // 0 options and size cannot grow

#if FEATUREMAP == NONLINEAR

const int NUM_HIDDEN = 10; //number of hidden units
const int NUM_GENES = NUM_BASE_FEATURES * NUM_HIDDEN + NUM_HIDDEN * NUM_BOTTOM_FEATURES + 2;
/* weights to construct bottom-level map features from base_features */
typedef Eigen::Matrix<float, NUM_HIDDEN, NUM_BASE_FEATURES, Eigen::DontAlign, NUM_HIDDEN, NUM_BASE_FEATURES> weight1_t;
typedef Eigen::Matrix<float, NUM_BOTTOM_FEATURES, NUM_HIDDEN, Eigen::DontAlign, NUM_BOTTOM_FEATURES, NUM_HIDDEN> weight2_t;
typedef Eigen::Matrix<float, NUM_HIDDEN, 1, Eigen::DontAlign, NUM_HIDDEN, 1> hidden_t;
struct Weights
{
    weight1_t W1;
    weight2_t W2;
    float B1, B2;
};
typedef Weights weight_t;
#else
const int NUM_GENES = NUM_BASE_FEATURES * NUM_BOTTOM_FEATURES;
/* weights to construct bottom-level map features from base_features */
typedef Eigen::Matrix<float, NUM_BOTTOM_FEATURES, NUM_BASE_FEATURES, Eigen::DontAlign, NUM_BOTTOM_FEATURES, NUM_BASE_FEATURES> weight_t;
#endif


/* bottom-level map features */
typedef Eigen::Matrix<float, NUM_BOTTOM_FEATURES, 1, Eigen::DontAlign, NUM_BOTTOM_FEATURES, 1> bottom_features_t;



#endif
