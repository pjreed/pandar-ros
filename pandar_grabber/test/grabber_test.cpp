#include "pandar_grabber/pandar_grabber.h"
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <string>
#include <iostream>
#include <iomanip>
 #include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>


class SimpleHDLGrabber 
{
  public:
    std::string calibrationFile, pcapFile;
	std::string ip_;
	int port_;
    pcl::visualization::CloudViewer viewer;
    pcl::PandarGrabber interface;

    SimpleHDLGrabber (std::string& calibFile, std::string& pcapFile) 
      : calibrationFile (calibFile)
      , pcapFile (pcapFile) 
		, ip_("")
		, port_(0)
      , interface (calibrationFile, pcapFile)
      ,viewer("Simple Cloud Viewer")
    {
       
    }

    SimpleHDLGrabber (std::string& calibFile, std::string& ip, int port) 
      : ip_(ip)
		, port_(port)
		, calibrationFile (calibFile)
      , pcapFile ("") 
      , interface (boost::asio::ip::address::from_string(ip), port, calibFile)
      ,viewer("Simple Cloud Viewer")
    {
       
    }

    void 
    rawCloud (
        const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >& cloud)
    {
      this->viewer.showCloud (cloud);
    }

    void 
    run () 
    {
      
      // make callback function from member function
      boost::function<void(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&)> f =
          boost::bind(&SimpleHDLGrabber::rawCloud, this, _1);
	   boost::signals2::connection c = interface.registerCallback(f);

      // start receiving point clouds
      interface.start ();

      std::cout << "<Esc>, \'q\', \'Q\': quit the program" << std::endl;
      char key;
      do 
      {
        key = static_cast<char> (getchar ());
      } while (key != 27 && key != 'q' && key != 'Q');

      // stop the grabber
      interface.stop ();
    }
};

int main(int argc , char** argv)
{
	std::string hdlCalibration("");
	std::string pcapFile("/home/ys/Downloads/pangu.pcap");
	std::string ip("");
	int port = 8080;

	pcl::console::parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
	pcl::console::parse_argument (argc, argv, "-pcapFile", pcapFile);
	pcl::console::parse_argument (argc, argv, "-ip", ip);
	pcl::console::parse_argument (argc, argv, "-port", port);

	std::cout<< "pcapFile: " << pcapFile <<std::endl;
	std::cout << "calibrationFile: " << hdlCalibration << std::endl;
	std::cout << "ip " << ip << " port " << port << std::endl;

	if (ip == "") {
		SimpleHDLGrabber grabber (hdlCalibration, pcapFile);
		grabber.run ();
	} else {
		SimpleHDLGrabber grabber (hdlCalibration, ip, port);
		grabber.run ();
	}

	return 0;
}
