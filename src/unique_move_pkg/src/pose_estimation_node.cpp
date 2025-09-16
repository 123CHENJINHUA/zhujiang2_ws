#include <ros/ros.h>
#include <robot_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "robot_msgs/Marker2dPose.h"

// Global publisher for Marker2dPose
ros::Publisher marker2dPosePub;
ros::Publisher bodyFoot2dPosePub;

void markerCallback(const robot_msgs::MarkerArray::ConstPtr& data, tf::TransformListener* tfListener) {
    // ROS_INFO("Received marker array with %lu markers", data->markers.size());
    for (const auto& marker : data->markers) {
        // ROS_INFO("Marker ID: %d, Confidence: %.2f", marker.id, marker.confidence);
        // ROS_INFO("Pose: [%f, %f, %f]", marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z);

        try {
            if (marker.id == 123) {
                // Transform marker pose from camera_link to map_3d
                geometry_msgs::PoseStamped markerPoseInCamera, markerPoseInMap;
                markerPoseInCamera.header.frame_id = "camera_link";
                markerPoseInCamera.header.stamp = ros::Time(0); // Use the latest transform
                markerPoseInCamera.pose = marker.pose.pose;

                // Perform the transformation
                tfListener->transformPose("map_3d", markerPoseInCamera, markerPoseInMap);

                // Extract 2D pose (x, y, theta)
                double x = markerPoseInMap.pose.position.x;
                double y = markerPoseInMap.pose.position.y;

                // Convert quaternion to yaw (theta)
                double theta = tf::getYaw(markerPoseInMap.pose.orientation) * 180.0 / M_PI; // Convert to degrees

                // ROS_INFO("Marker ID: %d, Pose in map_3d (2D): [x: %f, y: %f, theta: %f]", marker.id, x, y, theta);

                // Publish Marker2dPose
                robot_msgs::Marker2dPose marker2dPose;
                marker2dPose.header.frame_id = "map_3d";
                marker2dPose.header.stamp = ros::Time::now();
                marker2dPose.x = x;
                marker2dPose.y = y;
                marker2dPose.theta = theta;
                marker2dPosePub.publish(marker2dPose);
            }
        } catch (const tf::TransformException& ex) {
            // ROS_WARN("Failed to transform marker ID %d to map_3d: %s", marker.id, ex.what());
        }
    }
}

void bodyFootCallback(tf::TransformListener* tfListener) {
    try {
        // Lookup the transform from body_foot_2d to map_3d
        tf::StampedTransform transform;
        tfListener->lookupTransform("map_3d", "body_foot_2d", ros::Time(0), transform);

        // Extract 2D pose (x, y, theta)
        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double theta = tf::getYaw(transform.getRotation()) * 180.0 / M_PI; // Convert to degrees

        // ROS_INFO("Pose in map_3d (2D): [x: %f, y: %f, theta: %f]", x, y, theta);

        // Publish Marker2dPose
        robot_msgs::Marker2dPose bodyFoot2dPose;
        bodyFoot2dPose.header.frame_id = "map_3d";
        bodyFoot2dPose.header.stamp = ros::Time::now();
        bodyFoot2dPose.x = x;
        bodyFoot2dPose.y = y;
        bodyFoot2dPose.theta = theta;
        bodyFoot2dPosePub.publish(bodyFoot2dPose);
    } catch (const tf::TransformException& ex) {
        // ROS_WARN("Failed to transform body_foot_2d to map_3d: %s", ex.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "marker_subscriber");
    ros::NodeHandle nh;

    // Initialize TF listener
    tf::TransformListener tfListener;

    // Initialize Marker2dPose publisher
    marker2dPosePub = nh.advertise<robot_msgs::Marker2dPose>("/marker_2d_pose", 10);
    bodyFoot2dPosePub = nh.advertise<robot_msgs::Marker2dPose>("/body_foot_2d_pose", 10);

    // Subscribe to marker topic
    ros::Subscriber markerSub = nh.subscribe<robot_msgs::MarkerArray>(
        "/aruco_marker_publisher/markers", 10, boost::bind(markerCallback, _1, &tfListener));
    ROS_INFO("Subscribed to 'markers' topic");

     // Timer to periodically check body_foot_2d pose
     ros::Timer bodyFootTimer = nh.createTimer(ros::Duration(0.1), 
     boost::bind(bodyFootCallback, &tfListener));
    ROS_INFO("Started timer for 'body_foot_2d' pose transformation");

    // Keep the node running
    ros::spin();

    return 0;
}