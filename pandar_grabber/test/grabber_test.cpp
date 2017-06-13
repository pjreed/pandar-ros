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
    pcl::visualization::CloudViewer viewer;
    pcl::PandarGrabber interface;

    SimpleHDLGrabber (std::string& calibFile, std::string& pcapFile) 
      : calibrationFile (calibFile)
      , pcapFile (pcapFile) 
      , interface (calibrationFile, pcapFile)
      ,viewer("Simple Cloud Viewer")
    {
       
    }

    void 
    sectorScan (
        const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&, 
        float,
        float) 
    {

      static unsigned count = 0;
      static double last = pcl::getTime ();
      if (++count == 30) 
      {
        double now = pcl::getTime();
        std::cout << "got sector scan.  Avg Framerate " << double(count) / double(now - last) << " Hz" << std::endl;
        count = 0;
        last = now;
      }
    }

    void 
    sweepScan (
        const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >& sweep)
    {
    	// this->viewer.showCloud (sweep);
    	// sleep(10);
      static unsigned count = 0;
      static double last = pcl::getTime();
#if 0
      if (sweep->header.seq == 0) {
        pcl::uint64_t stamp;
        stamp = sweep->header.stamp;
        time_t systemTime = static_cast<time_t>(((stamp & 0xffffffff00000000l) >> 32) & 0x00000000ffffffff);
        pcl::uint32_t usec = static_cast<pcl::uint32_t>(stamp & 0x00000000ffffffff);
        std::cout << std::hex << stamp << "  " << ctime(&systemTime) << " usec: " << usec << std::endl;
      }

      if (++count == 30) 
      {
        double now = pcl::getTime ();
        std::cout << "got sweep.  Avg Framerate " << double(count) / double(now - last) << " Hz" << std::endl;
        count = 0;
        last = now;
      }
#else
		pcl::uint64_t stamp;
		stamp = sweep->header.stamp;
		time_t systemTime = static_cast<time_t>(((stamp & 0xffffffff00000000l) >> 32) & 0x00000000ffffffff);
		pcl::uint32_t usec = static_cast<pcl::uint32_t>(stamp & 0x00000000ffffffff);
		std::cout << "systemTime: " << systemTime
			<< " lidar timestamp: " << usec << std::endl;
#endif
    }

    void 
    rawScan (
        const boost::shared_ptr<const hesai::Scan >& scan)
    {


      pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
      interface.raw2PointCloud(*scan , point_cloud_ptr);
      this->viewer.showCloud (point_cloud_ptr);
      /*
      printf("sweep size %d\n", scan->sweeps_size());
      for(int i = 0 ; i < scan->sweeps_size() ; i ++)
      {
        printf("measure%d size %d\n", i , scan->sweeps(i).meas_size());
      }
      */
    }

    void 
    run () 
    {
      
      // make callback function from member function
      boost::function<void(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&, float, float)> f =
          boost::bind(&SimpleHDLGrabber::sectorScan, this, _1, _2, _3);
          // boost::signals2::connection c2 = interface.registerCallback(f);

      boost::function<void(const boost::shared_ptr<const hesai::Scan >&)> f1 =
          boost::bind(&SimpleHDLGrabber::rawScan, this, _1);
      boost::signals2::connection c3 = interface.registerCallback(f1);

      // connect callback function for desired signal. In this case its a sector with XYZ and intensity information
      // boost::signals2::connection c = interface.registerCallback(f);

      // Register a callback function that gets complete 360 degree sweeps.
      boost::function<void(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&)> f2 = boost::bind(
          &SimpleHDLGrabber::sweepScan, this, _1);
      boost::signals2::connection c2 = interface.registerCallback(f2);

      //interface.filterPackets(boost::asio::ip::address_v4::from_string("192.168.18.38"));

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

	pcl::console::parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
	pcl::console::parse_argument (argc, argv, "-pcapFile", pcapFile);

	std::cout<< pcapFile <<std::endl;

	SimpleHDLGrabber grabber (hdlCalibration, pcapFile);
	grabber.run ();
	return 0;
}
