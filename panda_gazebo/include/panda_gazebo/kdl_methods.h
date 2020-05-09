/***************************************************************************
* Methods from orocos_kdl library. Isolated to avoid memory leak bugs.
* Removing sns_ik_lib dependency

*
* @package: panda_gazebo
* @metapackage: panda_simulator
* @author: Saif Sidhik <sxs1412@bham.ac.uk>
*

**************************************************************************/

#ifndef PANDA_GAZEBO_KDL_METHODS_H
#define PANDA_GAZEBO_KDL_METHODS_H

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <kdl/chainfksolver.hpp>

#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>

namespace panda_gazebo
{
class KDLMethods{

public:

bool initialise(KDL::Chain& chain);
virtual int PosFKJntToCart(const KDL::JntArray& q_in, KDL::Frame& p_out, int seg_nr);
virtual int RNECartToJnt(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot, const KDL::Wrenches& f_ext,KDL::JntArray &torques, const KDL::Twist ag);

virtual int JacobianJntToJac(const KDL::JntArray& q_in, KDL::Jacobian& jac, int seg_nr=-1);

virtual int JntToCoriolis(const KDL::JntArray &q, const KDL::JntArray &q_dot, KDL::JntArray &coriolis);
virtual int JntToMass(const KDL::JntArray &q, KDL::JntSpaceInertiaMatrix& H);
virtual int JntToGravity(const KDL::JntArray &q,KDL::JntArray &gravity);

private:
    KDL::Chain chain_;
    unsigned int nj;
    unsigned int ns;
    std::vector<KDL::Frame> X;
    std::vector<KDL::Twist> S;
    std::vector<KDL::Twist> v;
    std::vector<KDL::Twist> a;
    std::vector<KDL::Wrench> f;
    KDL::Twist g_ag;
    KDL::Twist c_ag;

    KDL::Twist t_tmp;
    KDL::Frame T_tmp;
    std::vector<bool> locked_joints_;

    int nr;   
    KDL::Vector grav;
    KDL::Vector vectornull;
    KDL::JntArray jntarraynull;
    std::vector<KDL::Wrench> wrenchnull;
    std::vector<KDL::ArticulatedBodyInertia, Eigen::aligned_allocator<KDL::ArticulatedBodyInertia> > Ic;
    KDL::Wrench F;
};

} // namespace panda_gazebo

#endif  // PANDA_GAZEBO_KDL_METHODS_H