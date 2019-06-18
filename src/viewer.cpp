#include "object_tracking_2d_ros.h" 

using namespace std;

void marker_callback(visualization_msgs::MarkerArray marker_array){
    cout << "check\n";

}

void direction_callback(object_tracking_2d_ros::ObjectDetections object_detections){

}

int main(int argc, char ** argv){
    ros::init(argc, argv, "viewer");
    ros::NodeHandle n;
    //ros::Subscriber sub1 = n.subscribe("/object_tracking_2d_ros/marker_array", 1, marker_callback);
    //ros::Subscriber sub2 = n.subscribe("/object_tracking_2d_ros/detections", 1, direction_callback);
    ros::spin();
    return 0;
} 
