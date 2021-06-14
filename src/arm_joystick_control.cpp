/**
* @file moveit_arm_joystick_control.cpp
* @author Omkar Kabadagi
*/

#include<ros/ros.h>
#include<string>

#include<moveit/move_group_interface/move_group_interface.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>
#include<moveit/robot_model_loader/robot_model_loader.h>
#include<moveit/robot_model/robot_model.h>
#include<moveit/robot_state/robot_state.h>
#include<moveit_msgs/DisplayRobotState.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include<moveit_msgs/AttachedCollisionObject.h>
#include<moveit_msgs/CollisionObject.h>
#include<moveit_visual_tools/moveit_visual_tools.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<sensor_msgs/Joy.h>

namespace rvt = rviz_visual_tools;

boost::mutex joy_cmd_lock;

/**
* Square of a number with retaining the original sign
*
* @param value - number to be squared
* @return square of value with sign
*/
double signedSquare(double value)
{
    if(value>0)
        return value*value;
    else
        return -1*value*value;
}

//================
//Joystick Interface
//================
/**
* Implementation of Joystick Interface Base Class
*
* Joystick Interface defines the named inputs of gamepad.
* This helps generalize all gamepads and acts as an interface
* between actual controller mappings and general inputs
*
*/
class JoystickInterface
{
public:
    JoystickInterface() :
        center(false),
        select(false),
        start(false),
        cross(false),
        circle(false),
        square(false),
        triangle(false),
        L1(false),
        R1(false),
        L2(false),
        R2(false),
        L3(false),
        R3(false),
        left(false),
        right(false),
        up(false),
        down(false),
        left_analog_x(0.0),
        left_analog_y(0.0),
        right_analog_x(0.0),
        right_analog_y(0.0),
        buff_size(10){
    }

    void add(std::string);
    bool all(std::string);

public:
    bool center;
    bool select;
    bool start;
    bool cross;
    bool circle;
    bool square;
    bool triangle;
    bool L1;
    bool R1;
    bool L2;
    bool R2;
    bool L3;
    bool R3;
    bool left;
    bool right;
    bool up;
    bool down;
    double left_analog_x;
    double left_analog_y;
    double right_analog_x;
    double right_analog_y;

protected:
    int buff_size;
    std::vector<std::string> buff;
};

/**
* Add recent command to buffer
*
* @param joy_cmd - string equivalent of joystick command given
*/
void JoystickInterface::add(std::string joy_cmd)
{
    buff.push_back(joy_cmd);
    if(buff.size() > buff_size)
    {
        buff.erase(buff.begin());
    }
}

/**
* Check if a particular joystick command has filled the buffer
*
* @param joy_cmd - string equivalent of joystick command given
*/
bool JoystickInterface::all(std::string joy_cmd)
{
    for(auto cmd : buff)
        if(joy_cmd.compare(cmd) != 0)
            return false;

    return true;
}

//================
//Redgear Wired
//================
/**
* Implementation of Redgear Wired Controller
*
* Inherits from Joystick Interface Base Class and
* maps buttons and axes to named inputs of
* Joystick Interface
*/
class RedgearWired : public JoystickInterface
{
public:
    RedgearWired(ros::NodeHandle& nh) :
        nh(nh),
        button0(false),
        button1(false),
        button2(false),
        button3(false),
        button4(false),
        button5(false),
        button6(false),
        button7(false),
        button8(false),
        button9(false),
        button10(false),
        button11(false),
        button12(false),
        button13(false),
        button14(false),
        button15(false),
        button16(false),
        axis0(0.0),
        axis1(0.0),
        axis2(0.0),
        axis3(0.0),
        JoystickInterface()
    {
        joy_cmd_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &RedgearWired::joyCallback, this);
    }

    void joyCallback(const sensor_msgs::JoyConstPtr& msg);

protected:
    bool button0;
    bool button1;
    bool button2;
    bool button3;
    bool button4;
    bool button5;
    bool button6;
    bool button7;
    bool button8;
    bool button9;
    bool button10;
    bool button11;
    bool button12;
    bool button13;
    bool button14;
    bool button15;
    bool button16;
    double axis0;
    double axis1;
    double axis2;
    double axis3;

    ros::NodeHandle nh;
    ros::Subscriber joy_cmd_sub;
};

/**
* Callback for `/joy` topic
*
* @param msg - sensor message of Joy type carries joystick
    input information from topic
*/
void RedgearWired::joyCallback(const sensor_msgs::JoyConstPtr& msg)
{
    joy_cmd_lock.lock();

    if(msg->buttons[0])
    {
        cross = true;
        add("cross");
    }
    else
        cross = false;
    if(msg->buttons[1])
    {
        circle = true;
        add("circle");
    }
    else
        circle = false;
    if(msg->buttons[2])
    {
        square = true;
        add("square");
    }
    else
        square = false;
    if(msg->buttons[3])
    {
        triangle = true;
        add("triangle");
    }
    else
        triangle = false;
    if(msg->buttons[4])
    {
        L1 = true;
        add("L1");
    }
    else
        L1 = false;
    if(msg->buttons[5])
    {
        R1 = true;
        add("R1");
    }
    else
        R1 = false;
    if(msg->buttons[6])
    {
        L2 = true;
        add("L2");
    }
    else
        L2 = false;
    if(msg->buttons[7])
    {
        R2 = true;
        add("R2");
    }
    else
        R2 = false;
    if(msg->buttons[8])
    {
        select = true;
        add("select");
    }
    else
        select = false;
    if(msg->buttons[9])
    {
        start = true;
        add("start");
    }
    else
        start = false;
    if(msg->buttons[10])
    {
        center = true;
        add("center");
    }
    else
        center = false;
    if(msg->buttons[11])
    {
        L3 = true;
        add("L3");
    }
    else
        L3 = false;
    if(msg->buttons[12])
    {
        R3 = true;
        add("R3");
    }
    else
        R3 = false;
    if(msg->buttons[13])
    {
        left = true;
        add("left");
    }
    else
        left = false;
    if(msg->buttons[14])
    {
        right = true;
        add("right");
    }
    else
        right = false;
    if(msg->buttons[15])
    {
        up = true;
        add("up");
    }
    else
        up = false;
    if(msg->buttons[16])
    {
        down = true;
        add("down");
    }
    else
        down = false;

    left_analog_x = -msg->axes[0];
    left_analog_y = msg->axes[1];
    right_analog_x = -msg->axes[2];
    right_analog_y = msg->axes[3];

    joy_cmd_lock.unlock();
}

/**
* Implementation of Arm Interface Class
*
* Arm Interface controls the arm using moveit APIs. It stores
* all information about the kinematic model, and scene to
* to move the arm based on joystick input given
*/
class ArmInterface
{
public:
    ArmInterface(
        ros::NodeHandle& nh,
        std::string planning_group,
        std::string base_frame,
        robot_model::RobotModelPtr kinematic_model,
        std::string robot_description="robot_description"
    ) :
        planning_group(planning_group),
        base_frame(base_frame),
        robot_description(robot_description),
        move_group(new moveit::planning_interface::MoveGroupInterface(planning_group)),
        kinematic_model(kinematic_model),
        visual_tools(new moveit_visual_tools::MoveItVisualTools(base_frame)),
        red_gear(new RedgearWired(nh)){
    }

    ~ArmInterface()
    {
        delete red_gear;
    }

    void info();
    void run();

private:
    bool computeJoyPose();
    bool computeJoyPosePosition();
    bool computeJoyPoseOrientation();
    bool planAndExecute();

private:
    std::string planning_group;
    std::string base_frame;
    std::string robot_description;
    moveit::planning_interface::MoveGroupInterfacePtr move_group;
    moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_interface;
    robot_model::RobotModelPtr kinematic_model;
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools;

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    JoystickInterface *red_gear;
    geometry_msgs::PoseStamped joy_pose;
};

/**
* Prints basic information about the robot and scene
*/
void ArmInterface::info()
{
    ROS_INFO("Planning frame: %s", move_group->getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group->getEndEffectorLink().c_str());
    ROS_INFO("Available Planning Groups: ");
    std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    move_group->setEndEffectorLink(move_group->getEndEffectorLink().c_str());
    joy_pose = move_group->getCurrentPose();
}

/**
* Computes the target pose based on joystick inputs
*
* @return boolean value stating if the pose has changed wrt
*   the previous pose
*/
bool ArmInterface::computeJoyPose()
{
    bool is_position_changed = computeJoyPosePosition();
    bool is_orientation_changed = computeJoyPoseOrientation();

    ROS_DEBUG_STREAM(
        "--------\nPose:\n"<<"positon: ("<<joy_pose.pose.position.x<<", "<<joy_pose.pose.position.y<<", "<<joy_pose.pose.position.z<<")"<<std::endl
                 <<"orientation: ("<<joy_pose.pose.orientation.x<<", "<<joy_pose.pose.orientation.y<<", "<<joy_pose.pose.orientation.z<<", "<<joy_pose.pose.orientation.w<<")"<<std::endl
    );

    return is_position_changed || is_orientation_changed;
}

/**
* Computes the translation based on joystick inputs
*
* @return boolean value stating if the position has changed wrt
*   the previous pose
*/
bool ArmInterface::computeJoyPosePosition()
{
    double dx, dy, dz=0;
    double scale_analog = 10.0, scale_trigger = 4.0;

    joy_cmd_lock.lock();

    dx = signedSquare(-red_gear->left_analog_x)/scale_analog;
    dy = signedSquare(red_gear->left_analog_y)/scale_analog;

    if(red_gear->L2)
        dz = 0.005;
    else if(red_gear->R2)
        dz = -0.005;

    if(red_gear->all("L2") || red_gear->all("R2"))
        scale_trigger = 8.0;

    joy_cmd_lock.unlock();

    Eigen::Quaterniond q;
    q.x() = joy_pose.pose.orientation.x;
    q.y() = joy_pose.pose.orientation.y;
    q.z() = joy_pose.pose.orientation.z;
    q.w() = joy_pose.pose.orientation.w;
    Eigen::Matrix3d rotation_matrix = q.normalized().toRotationMatrix();
    Eigen::Vector3d translation_eef_frame(dx, dy, dz*scale_trigger);
    Eigen::Vector3d translation_base_frame = rotation_matrix * translation_eef_frame;

    joy_pose.pose.position.x += translation_base_frame(0);
    joy_pose.pose.position.y += translation_base_frame(1);
    joy_pose.pose.position.z += translation_base_frame(2);

    return translation_eef_frame(0)!=0.0 || translation_eef_frame(1)!=0.0 || translation_eef_frame(2)!=0.0;
}

/**
* Computes the rotation based on joystick inputs
*
* @return boolean value stating if the orientation has changed wrt
*   the previous pose
*/
bool ArmInterface::computeJoyPoseOrientation()
{
    double roll_z=0.0, pitch_x=0.0, yaw_y=0.0;
    double dtheta = 0.1;

    joy_cmd_lock.lock();

    if(red_gear->L1)
    {
        if(red_gear->all("L1"))
            yaw_y += dtheta*2;
        else
            yaw_y += dtheta;
    }
    if(red_gear->R1)
    {
        if(red_gear->all("R1"))
            yaw_y -= dtheta*2;
        else
            yaw_y -= dtheta;
    }
    if(red_gear->up)
    {
        if(red_gear->all("up"))
            pitch_x -= dtheta*2;
        else
            pitch_x -= dtheta;
    }
    if(red_gear->down)
    {
        if(red_gear->all("down"))
            pitch_x += dtheta*2;
        else
            pitch_x += dtheta;
    }
    if(red_gear->left)
    {
        if(red_gear->all("left"))
            roll_z -= dtheta*2;
        else
            roll_z -= dtheta;
    }
    if(red_gear->right)
    {
        if(red_gear->all("right"))
            roll_z += dtheta*2;
        else
            roll_z += dtheta;
    }

    joy_cmd_lock.unlock();

    tf2::Quaternion orientation;
    orientation.setX(joy_pose.pose.orientation.x);
    orientation.setY(joy_pose.pose.orientation.y);
    orientation.setZ(joy_pose.pose.orientation.z);
    orientation.setW(joy_pose.pose.orientation.w);

    tf2::Quaternion rotation_eef_frame;
    rotation_eef_frame.setRPY(pitch_x, yaw_y, roll_z);

    tf2::Quaternion rotation_base_frame;
    rotation_base_frame = orientation * rotation_eef_frame;
    geometry_msgs::Quaternion new_orientation = tf2::toMsg(rotation_base_frame);

    joy_pose.pose.orientation.x = new_orientation.x;
    joy_pose.pose.orientation.y = new_orientation.y;
    joy_pose.pose.orientation.z = new_orientation.z;
    joy_pose.pose.orientation.w = new_orientation.w;

    return pitch_x!=0.0 || yaw_y!=0.0 || roll_z!=0.0;
}

/**
* Plans and Executes the pose goal
*
* @return boolean value stating success/failure in planning
*/
bool ArmInterface::planAndExecute()
{
    move_group->setPoseTarget(joy_pose);
    move_group->setPlanningTime(2);
    bool success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success)
        ROS_ERROR_STREAM("PLANNING FAILED"<<std::endl);
    else
        move_group->execute(plan);

    return success;
}

/**
* Run starts the arm to take inputs from the controller,
* computes the pose based on controller inputs and
* executes the pose goal
*/
void ArmInterface::run()
{
    ROS_INFO_STREAM("START ARM CONTROL");

    while(true)
    {
        joy_cmd_lock.lock();
        if(red_gear->select)
            break;
        joy_cmd_lock.unlock();

        bool is_pose_changed = computeJoyPose();

        if(is_pose_changed)
            planAndExecute();

        ros::Duration(0.05).sleep();
    }

    ROS_INFO_STREAM("EXIT ARM CONTROL");
}

//================
//MAIN
//================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_arm_joystick_control");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    ArmInterface urc_arm(node_handle, "arm", "base_link", kinematic_model);
    urc_arm.info();

    urc_arm.run();

    ros::shutdown();
    return 0;
}
