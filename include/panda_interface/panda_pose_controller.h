// Copyright (c) 2017 Franka Emika GmbH

// HIRO Robotics Lab, CU Boulder, 2019

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/Pose.h>

#include "panda_utils/panda_trac_ik.h"

namespace hiro_panda
{
class PandaPoseController : public controller_interface::MultiInterfaceController<
                                hardware_interface::PositionJointInterface,
                                franka_hw::FrankaStateInterface>
{
public:
    // required functions for panda fci 
    bool init(hardware_interface::RobotHW *hw, ros::NodeHandle &nh) override;
    void update(const ros::Time &time, const ros::Duration &period) override;
    void starting(const ros::Time &time) override;
    void stopping(const ros::Time &time) override;

private:
    // hardware_interface and corresponding joint handles
    hardware_interface::PositionJointInterface *_position_joint_interface;
    std::vector<hardware_interface::JointHandle> _position_joint_handles;

    ros::Duration _elapsed_time;
    ros::Time _start_time;
    // target pose from user
    geometry_msgs::Pose _target_pose;

    ros::Subscriber _target_subscriber;

    std::array<double, 7> _joint_cmds;
    std::array<double, 7> _limited_joint_cmds;
    std::array<double, 7> _last_commanded_pos;

    // 7 by 4 matrix for coefficients for each franka panda joint
    std::array<std::array<double, 4>, 7> _vel_catmull_coeffs_first_spline;
    std::array<std::array<double, 4>, 7> _vel_catmull_coeffs_second_spline;
    std::array<double, 7> calc_max_pos_diffs;

    bool _is_executing_cmd;
    double _max_abs_vel = 2.1;
    // threshold for when it is acceptable to say robot has reached goal
    double _epsilon = 0.0001;
    // user selected trajectory method after inverse kinematics solver completes
    enum class TrajectoryMethod
    {
        ConstantVel = 1,
        TrapezoidVel,
        CatmullRomVel
    };

    TrajectoryMethod traj_method = TrajectoryMethod::ConstantVel;
    std::array<double, 7> _iters;
    std::array<double, 4> catmullRomSpline(const double &tau, const std::array<double,4> &points);

    hiro_panda::PandaTracIK _panda_ik_service;
    KDL::JntArray _joints_result;

    double calcSplinePolynomial(const std::array<double,4> &coeffs, const double &x);

    void targetCartesianPoseCb(const geometry_msgs::Pose &target_pose);

    void constantVelCmd(const double &min_step);

    void trapezoidVelCmd(const double &min_step, const double &v_change);

    void catmullRomSplineVelCmd(const double &norm_pos, const int &joint_num, const double &interval);

    bool _isGoalReached();

};

} // namespace hiro_panda