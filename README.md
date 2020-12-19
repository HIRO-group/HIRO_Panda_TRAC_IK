# Franka Panda TracIK Solver

A Cartesian pose controller in the form of a ROS package, developed at the HIRO Lab at CU Boulder.


## Details

Trac_IK, an IK solver with a solve rate of 99.88% for the Franka Emika Panda, is used to compute the 7 resulting joint angles for the arm. From there, the user can specify the trajectory method in which the robot moves from one pose to another.

There's an accompanying Jupyter Notebook ([here](src/splines.ipynb)) for Catmull-Rom Splines, which is the best (smoothest) trajectory method for the Panda in this repo. A short document about these Splines can be found [here](http://www.cs.cmu.edu/~462/www/projects/assn2/assn2/catmullRom.pdf).


**Note**: This package has been soley tested for the real robot, and doesn't work for the Franka Panda simulation. However, a posible future extension of this work would be for simulation!

## Installation, Building, and Usage

### Requires

* `trac_ik`: The inverse kinematics package compatible with Franka Panda (and a variety of other robot arms)
* `libfranka` and `franka_ros`: These libraries are needed to get started on the real Panda. For detailed instructions how to install, check out this link [here](https://frankaemika.github.io/docs/installation_linux.html) for installation.


### Steps

```sh
# create a new catkin workspace...

mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://bitbucket.org/traclabs/trac_ik.git
git clone https://github.com/HIRO-group/HIRO_Panda_TRAC_IK

cd ..

catkin build

source devel/setup.bash
```

## User Information

In order to run through the Inverse Kinematics, once the build command succeeds, to see IK in action:


```sh
source devel/setup.bash
```


```sh
roslaunch hiro_panda_ik hiro_panda_ik_controller.launch robot_ip:=<robot-url> trajectory_method:=<1,2,3 or 4>
```

Note: The trajectory method is the method which is used to move the arm after the inverse kinematics are calculated. The options are:
    * **1**: Constant velocity to each point (default option when `trajectory_method` is not provided)
    * **2**: Increase velocity at a constant rate, then maintain max rate, then decrease at constant rate once near end of trajectory ('trapezoid' method)
    * **3**: Catmull-Rom splines where velocity is calculated as a function of position (smoothest and best method)

To publish to the topic: 
```sh
rostopic pub -1 /hiro_panda/goto_pose geometry_msgs/Pose "{position:{ x: 0.4, y: 0.4, z: 0.5}, orientation:{ x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

The above example will not changed the orientation of the arm; it will move it to the desired 3-d coordinates. Make sure that the orientation (quarternion) normalizes to 1.
