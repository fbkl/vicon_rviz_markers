#include <ros/ros.h>
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher pub;

void callback(const vicon_bridge::MarkersPtr& msg)
{
	visualization_msgs::Marker m;

	m.header.stamp = ros::Time::now();
	m.header.frame_id = msg->header.frame_id;
	m.type = visualization_msgs::Marker::SPHERE;
	m.pose.orientation.w = 1;
	//m.duration=1;
	m.scale.x=0.01;
	m.scale.y=0.01;
	m.scale.z=0.01;
	m.color.r=0.80;
	m.color.g=0.80;
	m.color.b=0.80;
	m.color.a=1.0;
	visualization_msgs::MarkerArray mm;
	for (auto& marker:msg->markers)
	{
		m.ns= marker.marker_name;
		marker.translation.x/=1000;
		marker.translation.y/=1000;
		marker.translation.z/=1000;
		m.pose.position = marker.translation;
		mm.markers.push_back( m);
		m.id++;
	}
	
	pub.publish(mm);

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "viewer_of_markers");
  ros::NodeHandle nh;
      auto marker_sub = nh.subscribe("/vicon/markers", 10,callback);
      //auto markers_pub_ = nh.advertise<vicon_bridge::Markers>("/markers", 10);
	pub = nh.advertise<visualization_msgs::MarkerArray>("/marker_marker",10);

      ros::spin();


return 0;
}
