// Copyright (c) 2017 Franka Emika GmbH

// HIRO Robotics Lab, CU Boulder, 2019

#ifndef __PANDA_TRAC_IK_H__
#define __PANDA_TRAC_IK_H__

#include <trac_ik/trac_ik.hpp>

#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

#include <memory>
#include <string>

// class for tracik solver for panda robot

// TRAC_IK::TRAC_IK ik_solver(string base_link, string tip_link, string URDF_param="/robot_description",
// double timeout_in_secs=0.005, double error=1e-5, TRAC_IK::SolveType type=TRAC_IK::Speed);

namespace hiro_panda
{

class PandaTracIK
{
private:
    std::string _urdf_param_string;

    double _secs_timeout;
    double _error;

    int _num_steps;

    std::shared_ptr<TRAC_IK::TRAC_IK> _panda_trac_ik_solver;
    KDL::Chain _chain;
    std::shared_ptr<KDL::JntArray> _nominal_joint_arr;
    // min and max joint positions for each joint, in radians
    // from franka panda docs, since URDF might not be well defined 
    std::array<double, 7> _max_joint_positions {
        {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}
    };

    std::array<double, 7> _min_joint_positions {
        {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}
    };

public:
    explicit PandaTracIK();
    ~PandaTracIK();
    // member that determines if the result of trying tracik 10 times produced successful
    // result
    bool is_valid;
    KDL::JntArray perform_ik(const geometry_msgs::Pose &goto_pose);

    // bool getKDLLimits(KDL::JntArray &ll, KDL::JntArray &ul);
    // bool setKDLLimits(KDL::JntArray  ll, KDL::JntArray  ul);
};

} // namespace hiro_panda

#endif