#ifndef CLASS_CONTROL_H
#define CLASS_CONTROL_H

#include "ros/ros.h"
#include <vector>
#include "qcar_control/TrajectoryMessage.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"

struct States
{
    double North = 0.0;
    double East  = 0.0;
    double Psi   = 0.0;
    double Vel   = 0.0;
};

class classControl
{
public:
    classControl(ros::NodeHandle* _n);

    States* getStates();
    double getWPX(int idx);
    double getWPY(int idx);
    double getWPT(int idx);
    double getVel();
    std::vector<double>& getWPVec();
    int getIndex(double currentTime);
    int getNearestIndex(double X, double Y);
    int getNearestIndexForward(double X, double Y, int prev_idx, int window);
    void command(double omega, double delta);

private:
    ros::NodeHandle* n;
    ros::Subscriber subGuid;
    ros::Subscriber subNav;
    ros::Publisher pubCmdRl;
    ros::Publisher pubCmdRr;
    ros::Publisher pubCmdFl;
    ros::Publisher pubCmdFr;
    ros::Publisher pubCmdFls;
    ros::Publisher pubCmdFrs;

    std_msgs::Float32 velCmdR;
    std_msgs::Float32 velCmdL;
    std_msgs::Float64 angCmd;

    std::vector<double> waypoint_times;
    std::vector<double> waypoint_x;
    std::vector<double> waypoint_y;
    double velocity = 0.0;

    States qcarStates;

    void init_guidSub();
    void guidCallback(const qcar_control::TrajectoryMessage::ConstPtr& msg);
    void init_navSub();
    void navCallback(const nav_msgs::Odometry::ConstPtr& msg);
    double quat_to_rad(double x, double y, double z, double w);
    void init_cmdPub();
};

#endif
