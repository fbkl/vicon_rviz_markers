#include <ros/ros.h>
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
int main(int argc, char** argv)
{

  ros::init(argc, argv, "dummy_pub");
  ros::NodeHandle nh{"~"};
      //auto marker_pub_ = nh.advertise<vicon_bridge::Marker>("/marker", 10);

      std::string ns;
      nh.getParam("ns",ns);
      ROS_INFO_STREAM(ns);
      auto markers_pub_ = nh.advertise<vicon_bridge::Markers>(ns+"/markers", 10);
      ros::Rate r(10);
	while(ros::ok())
	{
		
		std::vector<vicon_bridge::Marker> this_marker{3};
          vicon_bridge::Markers these_markers;

	  this_marker[0].marker_name = "idk";
	  this_marker[0].translation.x = 0.98*1000;
	  this_marker[0].translation.y = 0.12*1000;
	  this_marker[0].translation.z = 0.03*1000;
	  
	  this_marker[1].marker_name = "idk2";
	  this_marker[1].translation.x = 0.98*1000;
	  this_marker[1].translation.y = 0.02*1000;
	  this_marker[1].translation.z = 0.13*1000;
	  
	  this_marker[2].marker_name = "idk3";
	  this_marker[2].translation.x = 0.98*1000;
	  this_marker[2].translation.y = 0.02*1000;
	  this_marker[2].translation.z = 0.03*1000;

	these_markers.markers = this_marker;
	  markers_pub_.publish(these_markers);

	  these_markers.header.stamp = ros::Time::now();
	r.sleep();

  ros::spinOnce();
	}


return 0;
}
