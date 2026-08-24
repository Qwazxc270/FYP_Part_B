// classControl.cpp
// control class for ROS control_package

#include "ros/ros.h"
#include "classControl.h"
#include <cmath>
#include <algorithm>

// Constructor
classControl::classControl(ros::NodeHandle* _n)
{
    n = _n;
    velocity = 0.0;
    qcarStates = States();
    init_guidSub();
    init_navSub();
    init_cmdPub();
}

// Guidance subscriber
void classControl::init_guidSub()
{
    subGuid = n->subscribe("/qcar/trajectory_topic", 0, &classControl::guidCallback, this);
}

void classControl::guidCallback(const qcar_control::TrajectoryMessage::ConstPtr& msg)
{
    waypoint_times = msg->waypoint_times;
    waypoint_x     = msg->waypoint_x;
    waypoint_y     = msg->waypoint_y;
    velocity       = msg->velocity;
}

// Waypoint accessors
double classControl::getWPX(int idx) { return (idx < waypoint_x.size()) ? waypoint_x.at(idx) : -1.0; }
double classControl::getWPY(int idx) { return (idx < waypoint_y.size()) ? waypoint_y.at(idx) : -1.0; }
double classControl::getWPT(int idx) { return (idx < waypoint_times.size()) ? waypoint_times.at(idx) : -1.0; }
double classControl::getVel() { return (waypoint_times.empty()) ? -1.0 : velocity; }
std::vector<double>& classControl::getWPVec() { return waypoint_times; }

// Navigation subscriber
void classControl::init_navSub()
{
    subNav = n->subscribe("/odom", 0, &classControl::navCallback, this);
}

void classControl::navCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    qcarStates.North = msg->pose.pose.position.y;
    qcarStates.East  = msg->pose.pose.position.x;
    qcarStates.Psi   = quat_to_rad(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    qcarStates.Vel   = std::sqrt(std::pow(msg->twist.twist.linear.x,2) + std::pow(msg->twist.twist.linear.y,2));
}

// States accessor
States* classControl::getStates() { return &qcarStates; }

// Quaternion -> yaw
double classControl::quat_to_rad(double x, double y, double z, double w)
{
    double a = 2.0 * (w * z + x * y);
    double b = 1.0 - 2.0 * (y*y + z*z); 
    return std::atan2(a, b);
}

// Command publisher
void classControl::init_cmdPub()
{
    pubCmdRl = n->advertise<std_msgs::Float32>("/wheelrl_motor/command", 1);
    pubCmdRr = n->advertise<std_msgs::Float32>("wheelrr_motor/command", 1);
    pubCmdFl = n->advertise<std_msgs::Float32>("/wheelfl_motor/command", 1);
    pubCmdFr = n->advertise<std_msgs::Float32>("wheelfr_motor/command", 1);
    pubCmdFls = n->advertise<std_msgs::Float64>("qcar/base_fl_controller/command", 1);
    pubCmdFrs = n->advertise<std_msgs::Float64>("qcar/base_fr_controller/command", 1);
}

void classControl::command(double omega, double delta)
{
    velCmdL.data = -omega;
    velCmdR.data = omega;
    angCmd.data = delta;

    pubCmdRl.publish(velCmdL);
    pubCmdRr.publish(velCmdR);
    pubCmdFl.publish(velCmdL);
    pubCmdFr.publish(velCmdR);

    pubCmdFls.publish(angCmd);
    pubCmdFrs.publish(angCmd);
}

int classControl::getNearestIndex(double X, double Y)
{
    if(waypoint_x.empty()) return 0;

    double best_dist = std::numeric_limits<double>::max();
    int best_idx = 0;
    for(size_t i = 0; i < waypoint_x.size(); i++)
    {
        double dx = waypoint_x[i] - X;
        double dy = waypoint_y[i] - Y;
        double d = dx*dx + dy*dy;
        if(d < best_dist) { best_dist = d; best_idx = i; }
    }
    return best_idx;
}


int classControl::getNearestIndexForward(double X, double Y, int prev_idx, int window)
{
    if(waypoint_x.empty()) return 0;

    int start = prev_idx;
    int end = std::min((int)waypoint_x.size() - 1, prev_idx + window);

    double best_dist = std::numeric_limits<double>::max();
    int best_idx = start;

    for(int i = start; i <= end; i++)
    {
        double dx = waypoint_x[i] - X;
        double dy = waypoint_y[i] - Y;
        double d = dx*dx + dy*dy;
        if(d < best_dist) { best_dist = d; best_idx = i; }
    }
    return best_idx;
}

// Safe index calculation
int classControl::getIndex(double currentTime)
{
    if(waypoint_times.empty()) return 0;

    auto it = std::upper_bound(waypoint_times.begin(), waypoint_times.end(), currentTime);
    int idx = std::distance(waypoint_times.begin(), it);
    return std::min(idx, (int)waypoint_times.size() - 1);
}
