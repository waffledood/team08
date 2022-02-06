#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <cstdlib>
#include "common.hpp"

double imu_ang_vel = -10, imu_lin_acc = 0; // unlikely to be spinning at -10 at the start
void cbImu(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu_ang_vel = msg->angular_velocity.z;
    imu_lin_acc = msg->linear_acceleration.x;
}

double wheel_l = 10, wheel_r = 10; // init as 10 bcos both are unlikely to be exactly 10 (both can start at non-zero if u reset the sim). whole doubles can also be exactly compared
void cbWheels(const sensor_msgs::JointState::ConstPtr &msg)
{
    wheel_l = msg->position[1]; // double check the topic. it is labelled there
    wheel_r = msg->position[0]; // double check the topic. it is labelled there
}

nav_msgs::Odometry msg_odom;
void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    msg_odom = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_motion");
    ros::NodeHandle nh;

    // Parse ROS parameters
    bool use_internal_odom;
    if (!nh.param("use_internal_odom", use_internal_odom, true))
        ROS_WARN(" TMOVE : Param use_internal_odom not found, set to true");
    bool verbose;
    if (!nh.param("verbose_motion", verbose, false))
        ROS_WARN(" TMOVE : Param verbose_motion not found, set to false");

    // Publisher
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 1, true);

    // Prepare published message
    geometry_msgs::PoseStamped pose_rbt;
    pose_rbt.header.frame_id = "world"; //for rviz

    if (use_internal_odom)
    { // subscribes to odom topic --> is the exact simulated position in gazebo; when used in real life, is derived from wheel encoders (no imu).
        // Subscriber
        ros::Subscriber sub_odom = nh.subscribe("odom", 1, &cbOdom);

        // initialise rate
        ros::Rate rate(25);

        // wait for dependent nodes to load (check topics)
        ROS_INFO("TMOTION: Waiting for topics");
        while (ros::ok() && nh.param("run", true) && msg_odom.header.seq == 0) // dependent on odom
        {
            rate.sleep();
            ros::spinOnce(); //update the topics
        }

        ROS_INFO("TMOTION: ===== BEGIN =====");

        // begin loop
        while (ros::ok() && nh.param("run", true))
        {
            // update topics
            ros::spinOnce();

            // write to published message
            pose_rbt.pose = msg_odom.pose.pose; //use msg_odom instead of pose_rbt later, cos u write the motion filter x,y, heading into pose_rbt. msg_odom is still ground truth, provided that u have put in the subscriber..aight

            // publish pose
            pub_pose.publish(pose_rbt);

            if (verbose)
            {
                // get ang_rbt from quaternion
                auto &q = pose_rbt.pose.orientation;
                double siny_cosp = 2 * (q.w * q.z + q.x * q.y); 
                double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);

                ROS_INFO("TMOTION: Pos(%7.3f, %7.3f)  Ang(%6.3f)  FVel(%6.3f)  AVel(%6.3f)",
                         pose_rbt.pose.position.x, pose_rbt.pose.position.y, atan2(siny_cosp, cosy_cosp),
                         msg_odom.twist.twist.linear.x, msg_odom.twist.twist.angular.z);
            }

            rate.sleep();
        }
    }
    else
    {
        // Parse additional ROS parameters
        Position pos_rbt;
        // nh.param() tries to retrieve the value of "initial_x" from NodeHandle nh & stores the value in
        // pos_rbt.x, but if it's not found, it stores a value of 0.0 in pos_rbt.x instead. 
        if (!nh.param("initial_x", pos_rbt.x, 0.0))
            ROS_WARN(" TMOVE : Param initial_x not found, set to 0.0");
        if (!nh.param("initial_y", pos_rbt.y, 0.0))
            ROS_WARN(" TMOVE : Param initial_y not found, set to 0.0");
        double wheel_radius;
        if (!nh.param("wheel_radius", wheel_radius, 0.033))
            ROS_WARN(" TMOVE : Param wheel_radius not found, set to 0.033");
        double axle_track;
        if (!nh.param("axle_track", axle_track, 0.16))
            ROS_WARN(" TMOVE : Param axle_track not found, set to 0.16");
        double weight_odom_v;
        if (!nh.param("weight_odom_v", weight_odom_v, 0.5))
            ROS_WARN(" TMOVE : Param weight_odom_v not found, set to 0.5");
        double weight_odom_w;
        if (!nh.param("weight_odom_w", weight_odom_w, 0.5))
            ROS_WARN(" TMOVE : Param weight_odom_w not found, set to 0.5");
        double weight_imu_v = 1 - weight_odom_v;
        double weight_imu_w = 1 - weight_odom_w;
        double straight_thresh;
        if (!nh.param("straight_thresh", straight_thresh, 0.05))
            ROS_WARN(" TMOVE : Param straight_thresh not found, set to 0.05");
        double motion_iter_rate;
        if (!nh.param("motion_iter_rate", motion_iter_rate, 50.0))
            ROS_WARN(" TMOVE : Param motion_iter_rate not found, set to 50");

        // Subscribers
        ros::Subscriber sub_wheels = nh.subscribe("joint_states", 1, &cbWheels);
        ros::Subscriber sub_imu = nh.subscribe("imu", 1, &cbImu);
        // TODO - Added Subscriber of "odom" 
        ros::Subscriber sub_odom = nh.subscribe("odom", 1, &cbOdom);

        // initialise rate
        ros::Rate rate(motion_iter_rate); // higher rate for better estimation

        // initialise message for publishing
        pose_rbt.header.frame_id = "world"; // for rviz to visualise model wrt space
        pose_rbt.pose.orientation.x = 0;    // 3 DOF robot; the default is zero anyway, so this line is unnecessary. but kept it here to highlight 3DOF
        pose_rbt.pose.orientation.y = 0;    // 3 DOF robot; the default is zero anyway, so this line is unnecessary. but kept it here to highlight 3DOF

        // wait for dependent nodes to load (check topics)
        ROS_INFO("TMOTION: Waiting for topics");
        while (ros::ok() && nh.param("run", true) && (wheel_l == 10 || wheel_r == 10 || imu_ang_vel == -10)) // dependent on imu and wheels
        {
            rate.sleep();
            ros::spinOnce(); //update the topics
        }

        ROS_INFO("TMOTION: ===== BEGIN =====");

        // declare / initialise other variables
        double ang_rbt = 0; // robot always start at zero.
        double prev_time = ros::Time::now().toSec();
        double dt = 0;

        ////////////////// DECLARE VARIABLES HERE //////////////////
        // variables relating to Odometry Motion Model 
        //double wheel_l_prev = 10, wheel_r_prev = 10;
        double wheel_l_prev = wheel_l, wheel_r_prev = wheel_r;
        double wheel_l_diff = 0, wheel_r_diff = 0;
        double linVel_odom = 0, angVel_odom = 0;

        // variables relating to Weighted Average Velocity
        double lin_vel = 0, ang_vel = 0;

        // variables relating to "Finding Displacements" 
        double turnRadius;
        double ang_rbt_prev = 0;
        double pos_rbt_x_prev = pos_rbt.x, pos_rbt_y_prev = pos_rbt.y;

        // debug variables 
        double odom_linVel = 0, odom_angVel = 0;
        double imu_linVel = 0, imu_angVel = 0;

        // loop
        while (ros::ok() && nh.param("run", true))
        {
            // update topics
            ros::spinOnce();

            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0) // ros doesn't tick the time fast enough
                continue;
            prev_time += dt;

            ////////////////// Odometry Motion Model //////////////////
            wheel_l_diff = wheel_l - wheel_l_prev;
            wheel_r_diff = wheel_r - wheel_r_prev;
            linVel_odom = (wheel_radius/2/dt) * (wheel_r_diff + wheel_l_diff);
            angVel_odom = (wheel_radius/axle_track/dt) * (wheel_r_diff - wheel_l_diff);
            // update wheel_l_prev & wheel_r_prev
            wheel_l_prev = wheel_l;
            wheel_r_prev = wheel_r;

            ////////////////// MOTION FILTER HERE //////////////////
            lin_vel = (weight_odom_v * linVel_odom + weight_imu_v * imu_lin_acc * dt) / (1 - weight_imu_v);
            ang_vel = (weight_odom_w * angVel_odom) + (weight_imu_w * imu_ang_vel);

            // debug variables 
            odom_linVel = linVel_odom;
            odom_angVel = angVel_odom;
            imu_linVel = odom_linVel + (imu_lin_acc * dt);
            imu_angVel = imu_ang_vel;

            ////////////////// Finding Displacements //////////////////
            ang_rbt = ang_rbt_prev + (ang_vel * dt);
            turnRadius = lin_vel / ang_vel;
            // computation of x_t & y_t
            if (abs(ang_vel) > straight_thresh) {
                pos_rbt.x = pos_rbt_x_prev + turnRadius * (-sin(ang_rbt_prev) + sin(ang_rbt));
                pos_rbt.y = pos_rbt_y_prev + turnRadius * (cos(ang_rbt_prev) - cos(ang_rbt));
            } else {
                pos_rbt.x = pos_rbt_x_prev + (lin_vel * dt * cos(ang_rbt_prev));
                pos_rbt.y = pos_rbt_y_prev + (lin_vel * dt * sin(ang_rbt_prev));
            }
            // updating of variables tracking previous variables' values & occurrences 
            ang_rbt_prev = ang_rbt;
            pos_rbt_x_prev = pos_rbt.x;
            pos_rbt_y_prev = pos_rbt.y;

            // publish the pose
            // inject position and calculate quaternion for pose message, and publish
            pose_rbt.pose.position.x = pos_rbt.x;
            pose_rbt.pose.position.y = pos_rbt.y;
            pose_rbt.pose.orientation.w = cos(ang_rbt / 2);
            pose_rbt.pose.orientation.z = sin(ang_rbt / 2);
            pub_pose.publish(pose_rbt);

            if (verbose)
            {
                ROS_INFO("Actual Motion: Pos(%7.3f, %7.3f)  Ang(%6.3f)",
                         pos_rbt.x, pos_rbt.y, ang_rbt);

                // ground truth 
                /*
                auto &q = pose_rbt.pose.orientation;
                double siny_cosp = 2 * (q.w * q.z + q.x * q.y); 
                double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
                ROS_INFO("TMOTION: Pos(%7.3f, %7.3f)  Ang(%6.3f)  FVel(%6.3f)  AVel(%6.3f)",
                         pose_rbt.pose.position.x, pose_rbt.pose.position.y, atan2(siny_cosp, cosy_cosp),
                         msg_odom.twist.twist.linear.x, msg_odom.twist.twist.angular.z);
                */
                // get ang_rbt from quaternion
                

                auto &q = msg_odom.pose.pose.orientation;
                double siny_cosp = 2 * (q.w * q.z + q.x * q.y); 
                double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);

                ROS_INFO("Ground Truth : Pos(%7.3f, %7.3f)  Ang(%6.3f)  FVel(%6.3f)  AVel(%6.3f)",
                         msg_odom.pose.pose.position.x, msg_odom.pose.pose.position.y, atan2(siny_cosp, cosy_cosp),
                         msg_odom.twist.twist.linear.x, msg_odom.twist.twist.angular.z);

                ROS_INFO("Just IMU: LinVel: %3.3f, AngVel: %6.3f", imu_linVel, imu_angVel);
                ROS_INFO("Encoders: LinVel: %3.3f, AngVel: %6.3f", odom_linVel, odom_angVel);
            }

            // sleep until the end of the required frequency
            rate.sleep();
        }
    }

    ROS_INFO("TMOTION: ===== END =====");
    return 0;
}
