#include "pandar_grabber/pandar_grabber.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/io.h>
#include <ros/ros.h>

class SimpleGrabber
{
public:
	SimpleGrabber(ros::NodeHandle& nh);
	~SimpleGrabber();
	void run();

private:
	ros::NodeHandle& nh_;
	std::string ip_;
	std::string frame_id_;
	std::string pcap_;
	int port_;

	void cloud_callback(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >& cloud);
	boost::shared_ptr<pcl::PandarGrabber> grabber_;

	ros::Publisher cloud_pub_;
};

int main(int argc , char** argv)
{
    ros::init(argc, argv, "pandar_node");
    ros::NodeHandle nh("~");
    SimpleGrabber grb(nh);
	grb.run();
    ros::spin();
    return 0;
}

SimpleGrabber::SimpleGrabber(ros::NodeHandle& nh):
	nh_(nh)
{
	nh_.param<std::string>("device_ip", ip_, "");
	nh_.param<std::string>("frame_id", frame_id_, "pandar");
	nh_.param<int>("port", port_, 2368);
	nh_.param<std::string>("pcap", pcap_, "");
	if (ip_ != "") {
		ROS_INFO_STREAM("grabber listening on " << ip_ << " port " << port_);
		grabber_.reset(new pcl::PandarGrabber(boost::asio::ip::address::from_string(ip_),
					port_, ""));
	} else if (pcap_ != "") {
		ROS_INFO_STREAM("no ip address spicified, using pcap...");
		ROS_INFO_STREAM(pcap_);
		grabber_.reset(new pcl::PandarGrabber("", pcap_));
	} else {
		ROS_ERROR_STREAM("no ip address spicified, no pcap file provided, what can I do for you...");
	}
//	cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pandar_points", 10);
	cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI> >("pandar_points", 10);
}

SimpleGrabber::~SimpleGrabber()
{
	if (grabber_)
		grabber_->stop();
}

void SimpleGrabber::cloud_callback(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >& cloud)
{
	//publish
	ROS_INFO_STREAM("got a cloud. size " << cloud->size());
	pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);
	points->points.resize(cloud->size());
	for (int i = 0; i < cloud->size(); i++) {
		const pcl::PointXYZI& spt = cloud->points[i];
		pcl::PointXYZI& dpt = points->points[i];
		dpt.x = spt.x;
		dpt.y = spt.y;
		dpt.z = spt.z;
		dpt.intensity = spt.intensity;
	}
	points->header.frame_id = frame_id_;
	points->header.stamp = cloud->header.stamp & 0x0000FFFFFFFFFFFF;
	ROS_INFO("points: %lx cloud: %lx", points->header.stamp, cloud->header.stamp);
	points->height = 1;
	points->width = points->size();
	cloud_pub_.publish(points);
}

void SimpleGrabber::run()
{
	boost::function<pcl::PandarGrabber::sig_cb_Hesai_Pandar_sweep_point_cloud_xyzi> f =
		boost::bind(&SimpleGrabber::cloud_callback, this, _1);
	boost::signals2::connection c = grabber_->registerCallback(f);
	grabber_->start();
}
