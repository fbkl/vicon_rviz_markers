#include <ros/ros.h>
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
#include "XmlRpcException.h"
#include "XmlRpcValue.h"

#include <cstdlib>
#include <ctime>
#include <cmath>

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

const std::string bar("\n======================================================\n");


class DummyMarkerGetter{
	public:
	std::vector<std::string> markerNames;
	ros::NodeHandle nh;
	std::vector<vicon_bridge::Marker> latest_marker_vec;

    // synthetic motion, so a live pipeline is distinguishable from a frozen one
    std::string motion = "yaw";         // none | yaw | sway | jitter
    double motion_frequency = 0.2;      // Hz
    double motion_amplitude_deg = 20.0; // yaw
    double motion_amplitude_m = 0.05;   // sway / jitter, BEFORE position_multiplier
    bool   motion_continuous = false;   // yaw: spin forever instead of oscillating
    double multiplier = 1.0;            // mm scaling from position_multiplier
    double cx = 0.0, cz = 0.0;          // marker centroid, so yaw turns the subject
    ros::Time t0;

	DummyMarkerGetter()
	{
	nh = ros::NodeHandle("~/marker");
	std::srand(static_cast<unsigned int>(std::time(nullptr)));
    t0 = ros::Time::now();

    nh.param<std::string>("motion", motion, motion);
    nh.param("motion_frequency", motion_frequency, motion_frequency);
    nh.param("motion_amplitude_deg", motion_amplitude_deg, motion_amplitude_deg);
    nh.param("motion_amplitude_m", motion_amplitude_m, motion_amplitude_m);
    nh.param("motion_continuous", motion_continuous, motion_continuous);
    ROS_WARN_STREAM(yellow << "AR: synthetic marker motion = " << motion << " ("
                    << motion_frequency << " Hz). marker/motion:=none for a static cloud."
                    << reset);
	
	try{	
		XmlRpc::XmlRpcValue markerList;
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
    if (!latest_marker_vec.empty()) {
        for (const auto& m : latest_marker_vec) { cx += m.translation.x; cz += m.translation.z; }
        cx /= latest_marker_vec.size();
        cz /= latest_marker_vec.size();
    }
	ROS_INFO("AR: Finished serring up markers");
	
	
	}

    /**
     * Returns a COPY of the marker cloud with the configured synthetic motion applied.
     *
     * The previous version built `a_marker_vec` with the jitter applied and then returned
     * `latest_marker_vec` -- the untouched original -- so the markers never moved. It also
     * used `std::rand()*0.01`, which is up to ~2.1e7, not the 10 cm the comment claimed.
     *
     * `latest_marker_vec` is the reference cloud and is never mutated, so the motion cannot
     * drift or accumulate over a long run.
     */
    std::vector<vicon_bridge::Marker> get_latest_marker(){

        // live-switchable: `rosparam set <node>/marker/motion none` takes effect next tick.
        // getParamCached subscribes to param updates, so this is not a master call per frame.
        std::string requested = motion;
        nh.getParamCached("motion", requested);
        if (requested != motion) {
            ROS_WARN_STREAM(yellow << "AR: synthetic marker motion " << motion << " -> "
                            << requested << reset);
            motion = requested;
            t0 = ros::Time::now(); // restart the phase, so yaw/sway resume from the rest pose
        }

        std::vector<vicon_bridge::Marker> out = latest_marker_vec;
        if (motion == "none" || out.empty())
            return out;

        const double t = (ros::Time::now() - t0).toSec();
        const double w = 2.0 * M_PI * motion_frequency;

        if (motion == "yaw") {
            // OpenSim ground is Y-up, so a heading change is a rotation about Y.
            // Rotate about the marker centroid, not the world origin, or the whole cloud
            // orbits the origin instead of the subject turning on the spot.
            const double a = motion_continuous
                           ? w * t
                           : (motion_amplitude_deg * M_PI / 180.0) * std::sin(w * t);
            const double c = std::cos(a), s_ = std::sin(a);
            for (auto& m : out) {
                const double x = m.translation.x - cx;
                const double z = m.translation.z - cz;
                m.translation.x = cx + c * x + s_ * z;
                m.translation.z = cz - s_ * x + c * z;
            }
        } else if (motion == "sway") {
            const double d = motion_amplitude_m * multiplier * std::sin(w * t);
            for (auto& m : out)
                m.translation.x += d;
        } else if (motion == "jitter") {
            const double a = motion_amplitude_m * multiplier;
            for (auto& m : out) {
                m.translation.x += a * (2.0 * std::rand() / RAND_MAX - 1.0);
                m.translation.y += a * (2.0 * std::rand() / RAND_MAX - 1.0);
                m.translation.z += a * (2.0 * std::rand() / RAND_MAX - 1.0);
            }
        } else {
            ROS_WARN_STREAM_THROTTLE(10, "AR: unknown marker/motion [" << motion
                                     << "], publishing a static cloud.");
        }

        return out;
    }
};
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
	ros::Rate r(100);
	ros::NodeHandle p_nh{"~/marker"};
	std::string world_tf_reference;
	p_nh.getParam("world_tf_reference", world_tf_reference);
	vicon_bridge::Markers these_markers;
	these_markers.header.frame_id= world_tf_reference;

	std::vector<vicon_bridge::Marker> this_marker;
	auto dMG = DummyMarkerGetter();

	while(ros::ok())
	{
		auto markerDefVec = dMG.get_latest_marker();

		these_markers.markers = markerDefVec;
		these_markers.header.stamp = ros::Time::now();

		markers_pub_.publish(these_markers);
		ros::spinOnce();

		r.sleep();

	}


	return 0;
}
