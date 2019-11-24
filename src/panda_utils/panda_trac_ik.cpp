// Copyright (c) 2017 Franka Emika GmbH

// HIRO Robotics Lab, CU Boulder, 2019

#include "panda_utils/panda_trac_ik.h"

namespace hiro_panda
{

PandaTracIK::PandaTracIK() : _urdf_param_string("/robot_description"), _secs_timeout(0.005),
                             _error(1e-6), _num_steps(10)
{
    std::string name = "panda";
    bool use_robot = true;
    if (!use_robot)
    {
        _panda_trac_ik_solver = nullptr;
        _nominal_joint_arr = nullptr;
        return;
    }
    _panda_trac_ik_solver = std::make_shared<TRAC_IK::TRAC_IK>(name + "_link0", name + "_hand", _urdf_param_string,
                                                               _secs_timeout, _error, TRAC_IK::Distance);

    KDL::JntArray ll, ul;

    if (!_panda_trac_ik_solver->getKDLChain(_chain))
    {
        ROS_ERROR("There was no valid KDL chain found");
        return;
    }

    if (!_panda_trac_ik_solver->getKDLLimits(ll, ul))
    {
        ROS_ERROR("There were no valid KDL joint limits found");
        return;
    }

    if (!(_chain.getNrOfJoints() == ll.data.size() && _chain.getNrOfJoints() == ul.data.size()))
    {
        ROS_ERROR("Invalid number of joints between chain and lower limit / upper limit");
        return;
    }

    // set joint limits in the case if urdf is not well defined

    for (int i = 0; i<7; i++) 
    {
        ul(i) = _max_joint_positions[i];
        ll(i) = _min_joint_positions[i];
    }

    _panda_trac_ik_solver->setKDLLimits(ll, ul);

    ROS_INFO("Using %d joints", _chain.getNrOfJoints());
    _nominal_joint_arr = std::make_shared<KDL::JntArray>(_chain.getNrOfJoints());

    // nominal joint positions halfway between upper and lower limits
    for (int j = 0; j < _nominal_joint_arr->data.size(); ++j)
    {
        _nominal_joint_arr->operator()(j) = (ll(j) + ul(j)) / 2.0;
    }
}

KDL::JntArray PandaTracIK::perform_ik(const geometry_msgs::Pose &goto_pose)
{
    // performs inverse kinematics on the given desired pose.
    int rc = -1;
    KDL::JntArray result;
    KDL::Frame pose_frame(KDL::Rotation::Quaternion(goto_pose.orientation.x,
                                                    goto_pose.orientation.y,
                                                    goto_pose.orientation.z,
                                                    goto_pose.orientation.w),
                          KDL::Vector(goto_pose.position.x,
                                      goto_pose.position.y,
                                      goto_pose.position.z));

    // inverse kinematics, _num_steps trials   
    is_valid = false;                  
    for (int _num = 0; _num < _num_steps; ++_num)
    {
        ROS_INFO_STREAM("Attempting number " << _num+1 << " attempt for TracIK");
        rc = _panda_trac_ik_solver->CartToJnt(*(_nominal_joint_arr), pose_frame, result);
        if (rc >= 0)
        {
            ROS_INFO("Found a solution to desired pose");
            is_valid = true;
            break;
        }
            
    }
    if (!is_valid)
    {
        ROS_ERROR("Unable to find valid IK solution!");
    }
    
    return result;
}

PandaTracIK::~PandaTracIK()
{
}

} // namespace hiro_panda