#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/ros.h>

#include <stdio.h>
#include <cmath>
#include <iostream>
#include <sstream>

#define _USE_MATH_DEFINES

const float SAMPLE_TIME_S       = 0.01;

typedef struct obj_list
{
    char  objeto;
    float x;
    float y;
    float z;
    float orientation;
    float radio;  
} obj_list;

class ObstacleSimulator
{
    public:
        
        obj_list lista_objetos[1];
        float max_visible_radius;
        float fov_angle;
        float fov_slope;
        float fov_x_offset;

        ros::NodeHandle     nh_;
        ros::Publisher      marker_pub;
        ros::Publisher      detector_pub;
    
    ObstacleSimulator(int challenge_select, ros::NodeHandle nh)
    {
        this->nh_ = nh;
        this->marker_pub = this->nh_.advertise<visualization_msgs::MarkerArray>("/object_simulator/obstacle_markers", 1000);
        // Cylinder
        this->lista_objetos[0].objeto         = 'g';
        this->lista_objetos[0].x              = 0.30; 
        this->lista_objetos[0].y              = 0.0; 
        this->lista_objetos[0].z              = 0.0; 
        this->lista_objetos[0].orientation    = 0.52;
        this->lista_objetos[0].radio          = 0.10;
    }

    void rviz_markers()
    {
        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray marker_array;                            
        marker.header.frame_id  = "/world";
        marker.lifetime = ros::Duration(0.1);
        //Primer poste gate
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = this->lista_objetos[0].x;
        marker.pose.position.y = -this->lista_objetos[0].y;
        marker.pose.position.z = -this->lista_objetos[0].z+0.05;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.0762;
        marker.scale.y = 0.0762;
        marker.scale.z = 0.10;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.id = 0;
        marker_array.markers.push_back(marker);
        this->marker_pub.publish(marker_array);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "displaycylinder"); 
    ros::NodeHandle     n;
    ObstacleSimulator   obstacleSim(0, n);
    ros::Rate           loop_rate(1.0/SAMPLE_TIME_S);
    
    while(ros::ok())
    {
        ros::spinOnce();

        obstacleSim.rviz_markers();
    
        loop_rate.sleep();
    } 
    return 0;
}