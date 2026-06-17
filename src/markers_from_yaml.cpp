#include <ros/ros.h>
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
#include "XmlRpcException.h"
#include "XmlRpcValue.h"




const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

const std::string bar("\n======================================================\n");

std::vector<vicon_bridge::Marker> get_latest_marker(){

	std::vector<std::string> markerNames;
	ros::NodeHandle nh{"~/marker"};
	std::vector<vicon_bridge::Marker> latest_marker_vec;

	try{	
		XmlRpc::XmlRpcValue markerList;
		double multiplier =1.0;
		nh.getParam("position_multiplier", multiplier);
		nh.getParam("observation_order", markerList);
		if(markerList.valid())
			ROS_WARN_STREAM("AR markerObservationOrder:" << markerList.toXml());
		else 
			throw(XmlRpc::XmlRpcException("Couldn't parse observation_order"));
		//ROS_ASSERT(markerList.getType() == XmlRpc::XmlRpcValue::TypeArray); //

		if (markerList.size() == 0)
		{
			ROS_FATAL("AR Marker observation order not defined!");
			throw(std::invalid_argument("markerNames not defined."));
		}
		else
		{
			ROS_INFO_STREAM("AR: parsing points and marker vector");
			for (int32_t i = 0; i < markerList.size(); ++i) 
			{
				ROS_INFO("AR: Entered loop");
				XmlRpc::XmlRpcValue markerDef = markerList[i]; 
				ROS_INFO("AR: Loaded MarkerDef");
				std::string this_marker_name = markerDef["marker_name"];
				ROS_INFO_STREAM("AR: Assigned name: " << this_marker_name);
				markerNames.push_back(this_marker_name);
				ROS_INFO_STREAM("AR: ADDED TO MARKER LIST: " << magenta << markerNames.back() );
				//We also add the default position of the marker so that if the AR is broken, it doesnt affect the measurement too much
				XmlRpc::XmlRpcValue this_marker_default_position = markerDef["default_position"];
				ROS_INFO_STREAM(cyan <<"read default_position ok"<<reset);
				ROS_ASSERT(this_marker_default_position.getType() == XmlRpc::XmlRpcValue::TypeArray); //
				double x = this_marker_default_position[0];
				double y = this_marker_default_position[1];
				double z = this_marker_default_position[2];
				ROS_INFO_STREAM(cyan <<"setting x,y,z okay"<<reset);
				vicon_bridge::Marker this_marker;		

				this_marker.translation.x = x*multiplier;
				this_marker.translation.y = y*multiplier;
				this_marker.translation.z = z*multiplier;
				this_marker.marker_name = this_marker_name;
				latest_marker_vec.push_back(this_marker);


				ROS_INFO_STREAM(cyan <<"FINISHED SETTING UP ONE MARKER AT LEAST"<<reset);
				//ROS_ASSERT(markerDef[i].getType() == XmlRpc::XmlRpcValue::TypeString);
			}
			//for (auto& marker:markerNames)
			//	marker+=tf_frame_prefix;
		}
	}
	catch(XmlRpc::XmlRpcException& e)
	{
		ROS_ERROR_STREAM("AR: Could not setup markers" << e.getMessage());
	}
	ROS_INFO("AR: Finished serring up markers");

	return latest_marker_vec;
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "dummy_pub");
	//ros::NodeHandle nh{"~"};
	ros::NodeHandle nh;

	//auto marker_pub_ = nh.advertise<vicon_bridge::Marker>("/marker", 10);

	std::string ns;
	nh.getParam("ns",ns);
	ROS_INFO_STREAM(ns);
	//auto markers_pub_ = nh.advertise<vicon_bridge::Markers>(ns+"/markers", 10);
	auto markers_pub_ = nh.advertise<vicon_bridge::Markers>("markers", 10);
	ros::Rate r(10);
	ros::NodeHandle p_nh{"~/marker"};
	std::string world_tf_reference;
	nh.getParam("world_tf_reference", world_tf_reference);
	vicon_bridge::Markers these_markers;
	these_markers.header.frame_id= world_tf_reference;

	std::vector<vicon_bridge::Marker> this_marker;

	auto markerDefVec = get_latest_marker();


	these_markers.markers = markerDefVec;
	while(ros::ok())
	{
		these_markers.header.stamp = ros::Time::now();

		markers_pub_.publish(these_markers);
		ros::spinOnce();

		r.sleep();

	}


	return 0;
}
