#ifndef BOTTOM_TYPEDEFS_HPP
#define BOTTOM_TYPEDEFS_HPP



#include <planar_dart/descriptors.hpp>
#include <sferes/eval/eval.hpp>

#include <meta-cmaes/params.hpp>


#include <meta-cmaes/global.hpp>

#include <boost/serialization/vector.hpp> // serialising database vector

#include <boost/serialization/array.hpp>

#include <boost/fusion/container/vector.hpp>


#include <meta-cmaes/sampled.hpp>
#include <stdexcept>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// BOTTOM
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// // bottom-level typedefs
//typedef sferes::eval::Eval<BottomParams> bottom_eval_t;

typedef sferes::gen::Sampled<8, BottomParams> bottom_gen_t; // 8 parameters for our controller
typedef size_t bottom_gen_data_t;                            // sampled data type is based on unsigned ints
typedef boost::fusion::vector<> base_safe_t;
typedef boost::fusion::vector<planar_dart::descriptors::PositionalCoord, planar_dart::descriptors::PolarCoord,  planar_dart::descriptors::JointPairAngle,  planar_dart::descriptors::RelativeJointPairAngle, planar_dart::descriptors::AngleSum> base_desc_t;
typedef planar_dart::control base_controller_t;
typedef planar_dart::planarDARTSimu<planar_dart::safety<base_safe_t>, planar_dart::desc<base_desc_t>> simulator_t;
#endif
