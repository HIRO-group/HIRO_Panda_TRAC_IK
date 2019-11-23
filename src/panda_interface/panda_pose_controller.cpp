// Copyright (c) 2017 Franka Emika GmbH

// HIRO Robotics Lab, CU Boulder, 2019

#include "panda_interface/panda_pose_controller.h"

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace hiro_panda
{
bool PandaPoseController::init(hardware_interface::RobotHW *hw, ros::NodeHandle &nh)
{
    _target_subscriber = nh.subscribe("/hiro_panda/goto_pose", 10, &PandaPoseController::targetCartesianPoseCb, this,
                                      ros::TransportHints().reliable().tcpNoDelay());

    // get parameters and trajectory method selection
    int traj_method_int = 1;
    nh.getParam("/hiro_panda/trajectory_method", traj_method_int);
    traj_method = (TrajectoryMethod) traj_method_int;
    // limiting acceleration
    for (int i = 0; i < _lower_max_acceleration.size(); i++)
    {
        _lower_max_acceleration[i] = franka::kMaxJointAcceleration[i] * 0.1;
    }

    std::string arm_id;
    if (!nh.getParam("arm_id", arm_id))
    {
        ROS_ERROR("PandaPoseController: Could not get parameter arm_id");
        return false;
    }
    
    std::vector<std::string> joint_names;
    if (!nh.getParam("joint_names", joint_names))
    {
        ROS_ERROR("PandaPoseController: Could not parse joint names");
        return false;
    }

    if (joint_names.size() != 7)
    {
        ROS_ERROR_STREAM("PandaPoseController: Wrong number of joint names, got "
                         << joint_names.size() << " instead of 7 names!");
        return false;
    }

    _position_joint_interface = hw->get<hardware_interface::PositionJointInterface>();
    if (_position_joint_interface == nullptr)
    {
        ROS_ERROR("PandaPoseController: Could not initialize position joint interface");
        return false;
    }
    // position joint handles for position control
    _position_joint_handles.resize(7);
    for (int i = 0; i < 7; i++)
    {
        try
        {
            _position_joint_handles[i] = _position_joint_interface->getHandle(joint_names[i]);
        }

        catch (const hardware_interface::HardwareInterfaceException &e)
        {
            ROS_ERROR_STREAM(
                "PandaPoseController: Exception getting joint handles: " << e.what());
            return false;
        }
    }

    _state_interface = hw->get<franka_hw::FrankaStateInterface>();
    if (_state_interface == nullptr)
    {
        ROS_ERROR("PandaPoseController: Could not get Franka State Interface");
        return false;
    }

    try
    {
        _state_handle = std::make_unique<franka_hw::FrankaStateHandle>(_state_interface->getHandle(arm_id + "_robot"));
    }
    catch (const hardware_interface::HardwareInterfaceException &e)
    {
        ROS_ERROR_STREAM("PandaPoseController: Could not get Franka State Interface Handle" << e.what());
        return false;
    }
    // hiro panda trac ik service
    _panda_ik_service = hiro_panda::PandaTracIK();
    _is_executing_cmd = false;
    // max of n seconds for a move for catmull rom spline via position
    calc_max_time = 11.;
    // max velocity in rads/s
    max_vel = _max_abs_vel;
    // get the ik service once interface checks and everything are good to go
    return true;
    // get state interface and handle
}

void PandaPoseController::starting(const ros::Time &time)
{
    _joints_result.resize(7);
    for (int i = 0; i < 7; i++)
    {
        _joints_result(i) = _position_joint_handles[i].getPosition();
        _last_commanded_pos[i] = _joints_result(i); 
        _iters[i] = 0;
    }
}

void PandaPoseController::update(const ros::Time &time, const ros::Duration &period)
{
    double interval_length = period.toSec();
    // get joint commands in radians from inverse kinematics
    for (int i = 0; i < 7; i++)
    {
        _joint_cmds[i] = _joints_result(i);
    }

    double step = 0.0003;
    double max_velo = 0.0003;
    double v_change = 0.00003;
    switch (traj_method)
    {
        case TrajectoryMethod::ConstantVel:
            {
                // maintain slow, constant velocity during trajectory
                constantVelCmd(step);
            }
            break;
        
        case TrajectoryMethod::TrapezoidVel:
            {
                // increase velocity linearly to a certain point, maintain it,
                // then linearly decrease to 0 near the end of the motion
                trapezoidVelCmd(max_velo, v_change);
            }
            break;

        case TrajectoryMethod::CatmullRomPos:
            {
                // normalize time
                // get current time minus this time
                double time_diff = time.toSec() - _start_time.toSec();
                // normalize value between 0 and 1 based on max time for a spline move
                double t_norm = time_diff / calc_max_time;
                // plug into catmull rom position equation
                catmullRomSplinePosCmd(t_norm);
            }
            break;

        case TrajectoryMethod::CatmullRomVel:
            {
                // best and most recommended method for trajectory computation after
                // inverse kinematics calculations
                
                // normalize position, we need to do this separately for every joint
                double current_pos, p_val;
                for (int i=0; i<7; i++)
                {
                    current_pos = _position_joint_handles[i].getPosition();
                    // norm position
                    p_val = 2 - (2 * (abs(_joint_cmds[i] - current_pos) / abs(calc_max_pos_diffs[i])));
                    p_val = std::max(p_val, 0.);
                    catmullRomSplineVelCmd(p_val, i, interval_length);
                }
            }
            break;
    }

    for (int i = 0; i < 7; i++)
    {
        _position_joint_handles[i].setCommand(_limited_joint_cmds[i]);
    }

    franka::RobotState robot_state = _state_handle->getRobotState();
    
    for (int i = 0; i < 7; i++)
    {
        _last_commanded_pos[i] = _position_joint_handles[i].getPosition();
    }
}

void PandaPoseController::stopping(const ros::Time &time)
{
    // not implemented yet, can't send immediate commands for 0 velocity to robot
}

void PandaPoseController::targetCartesianPoseCb(const geometry_msgs::Pose &target_pose)
{
    // if (_is_executing_cmd) 
    // {
    //     ROS_ERROR("Panda Pose Controller: Still executing command!");
    //     return;
    //     // panda is still executing command
    // }
    _target_pose.orientation.w = target_pose.orientation.w;
    _target_pose.orientation.x = target_pose.orientation.x;
    _target_pose.orientation.y = target_pose.orientation.y;
    _target_pose.orientation.z = target_pose.orientation.z;

    _target_pose.position.x = target_pose.position.x;
    _target_pose.position.y = target_pose.position.y;
    _target_pose.position.z = target_pose.position.z;

    // use tracik to get joint positions from target pose
    KDL::JntArray ik_result = _panda_ik_service.perform_ik(_target_pose);
    _joints_result = (_panda_ik_service.is_valid) ? ik_result : _joints_result; 
    
    if (_joints_result.rows() != 7)
    {   
        ROS_ERROR("Panda Pose Controller: Wrong Amount of Rows Received From TRACIK");
        return;
    }
    
    std::array<double,4> position_points;
    std::array<double,4> velocity_points_first_spline;
    std::array<double,4> velocity_points_second_spline;

    for (int i=0; i<7; i++)
    {
        max_vel = _max_abs_vel;
        // get current position
        double cur_pos = _position_joint_handles[i].getPosition();
        // difference between current position and desired position from ik
        calc_max_pos_diffs[i] = _joints_result(i) - cur_pos;
        // if calc_max_poss_diff is negative, flip sign of max vel
        max_vel = calc_max_pos_diffs[i] < 0 ? -max_vel : max_vel;
        int sign = calc_max_pos_diffs[i] < 0 ? -1 : 1;
        // get p_i-2, p_i-1, p_i, p+i+1 for catmull rom
        position_points = {cur_pos - calc_max_pos_diffs[i], cur_pos, 
                            _joints_result(i), _joints_result(i) + calc_max_pos_diffs[i]};
        velocity_points_first_spline = {0, 0.3*sign, max_vel, max_vel};
        velocity_points_second_spline = {max_vel, max_vel, 0, 0};

        // compute spline on one joint i 
        // tau of 0.3
        
        _pos_catmull_coeffs[i] = catmullRomSpline(0.3, position_points);

        // separate splines for working with velocity as function of position
        _vel_catmull_coeffs_first_spline[i] = catmullRomSpline(0.3, velocity_points_first_spline);
        _vel_catmull_coeffs_second_spline[i] = catmullRomSpline(0.3, velocity_points_second_spline);
        
    }
    // _pos_catmull_coeffs has all coefficients for all joints now
    _is_executing_cmd = true;
    for (int i = 0; i < 7; i++)
    {
        _iters[i] = 0;
    }
    _start_time = ros::Time::now();
}

std::array<double, 4> PandaPoseController::catmullRomSpline(const double &tau, const std::array<double,4> &points)
{
    // catmullRomSpline calculation for any 4 generic points
    // result array for 4 coefficients of cubic polynomial
    std::array<double, 4> coeffs;
    // 4 by 4 matrix for calculating coefficients
    std::array<std::array<double, 4>, 4> catmullMat = {{{0, 1, 0, 0}, {-tau, 0, tau, 0},
                                                        {2*tau, tau-3, 3 - (2*tau), -tau}, 
                                                        {-tau,2-tau,tau-2,tau}}};
    // matrix-vector multiplication
    for (int i=0; i<4; i++)
    {
        coeffs[i] = (points[0]*catmullMat[i][0]) + (points[1]*catmullMat[i][1]) 
                    + (points[2]*catmullMat[i][2]) + (points[3]*catmullMat[i][3]);
    }
    return coeffs;
    
}
double PandaPoseController::calcSplinePolynomial(const std::array<double,4> &coeffs, const double &x)
{
    // function that calculates third degree polynomial given input x and 
    // 4 coefficients
    double output = 0.;   
    int power = 0;
    for (int i=0; i<4; i++)
    {
        output+=(coeffs[i]*(pow(x, power)));
        power++;
    }
    return output;
}

void PandaPoseController::constantVelCmd(const double &min_step)
{
    // reduce the commands that are sent to the robot in order to limit the rate
    // in which commands are sent at 1khz.

    // generate waypoints

    for (int i = 0; i < 7; i++)
    {
        // difference between joint command and position, will be big initially 
        double diff = _joint_cmds[i] - _position_joint_handles[i].getPosition();
        if (abs(diff) > min_step)
        {
            // difference is too big, reduce to min step instead, min step is rad/s
            _limited_joint_cmds[i] = _last_commanded_pos[i] + min_step * (diff > 0 ? 1.0 : -1.0);
        }
        else    
        {
            // difference is small, near the end send the raw command
            _limited_joint_cmds[i] = _joint_cmds[i];
        }
    }                             
}

void PandaPoseController::trapezoidVelCmd(const double &max_step, const double &v_change)
{
    // generates waypoints that increase velocity, then decrease
    double time = max_step / v_change;
    // time to go from 0 to min_step for each time step
    double change_dist = time * max_step * 0.5;

    for (int i = 0; i < 7; i++)
    {
        double diff = _joint_cmds[i] - _position_joint_handles[i].getPosition();

        if (_iters[i] == 0 && abs(diff) <= change_dist)
        {
            // case where we have new trajectory, but it doesn't need the whole path
            _limited_joint_cmds[i] = _joint_cmds[i];
        }
        
        else if (abs(diff) <= change_dist)
        {
            // when distance is small enough, decrease _iter value to 0 in decreased velocity 
            _iters[i] -= v_change;
            _iters[i] = std::max(_iters[i], 0.);
            _limited_joint_cmds[i] = _last_commanded_pos[i] + _iters[i] * (diff > 0 ? 1.0 : -1.0);

        }
        else if (abs(diff) > change_dist)
        {
            _iters[i] += v_change;
            _iters[i] = std::min(_iters[i], max_step);
            _limited_joint_cmds[i] = _last_commanded_pos[i] + _iters[i] * (diff > 0 ? 1.0 : -1.0);
        }
    }
}

void PandaPoseController::catmullRomSplinePosCmd(const double &norm_time)
{
    // command only executes when executing command
    // position is expressed as a function of time 
    if (norm_time <= 1 && _is_executing_cmd)
    {
        for (int i=0; i<7; i++)
        {
            double pos = calcSplinePolynomial(_pos_catmull_coeffs[i], norm_time);
            _limited_joint_cmds[i] = pos;

            // go through each joint and get the desired position from the normalized time
        }
        // calculate position value from time (should already be scaled)
    }
    else
    {
        for (int i=0; i<7; i++)
        {
            _limited_joint_cmds[i] = _joint_cmds[i];
        }
        _is_executing_cmd = false;
    }
    
}

void PandaPoseController::catmullRomSplineVelCmd(const double &norm_pos, const int &joint_num,
                                                    const double &interval)
{
    // individual joints
    // velocity is expressed as a function of position (position is normalized)
    if (norm_pos <= 2 && _is_executing_cmd)
    {
        double vel;
        if (norm_pos < 1)
        {
            // use first spline to get velocity
            vel = calcSplinePolynomial(_vel_catmull_coeffs_first_spline[joint_num], norm_pos);

        }
        else 
        {
            // use second spline to get velocity
            vel = calcSplinePolynomial(_vel_catmull_coeffs_second_spline[joint_num], norm_pos - 1);
        }
        
        // calculate velocity step
        _limited_joint_cmds[joint_num] = _last_commanded_pos[joint_num] + (vel * interval);
    }
    else
    {
        for (int i=0; i<7; i++)
        {
            _limited_joint_cmds[i] = _joint_cmds[i];
        }
        
        _is_executing_cmd = false;
    }
    
}

} // namespace hiro_panda

PLUGINLIB_EXPORT_CLASS(hiro_panda::PandaPoseController,
                       controller_interface::ControllerBase)
