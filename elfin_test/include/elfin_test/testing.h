#include "elfin_kinematic_solver/elfin_kinematic_solver.h"
#include <trac_ik/trac_ik.hpp>
#include <time.h>
#include <eigen3/Eigen/Eigen>
//#include <rand>

#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics