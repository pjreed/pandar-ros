/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2012,2015 The MITRE Corporation
 *  Copyright (c) 2017 Hesai Photonics Technology Co., Ltd
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/console/print.h>
#include <pcl/io/boost.h>
#include "pandar_grabber/pandar_grabber.h"
#include <boost/version.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/math/special_functions.hpp>
#include <boost/filesystem.hpp>
#include <pcap.h>
#include <fstream>
#include <sstream>

/*
 *   Constant Definition
 */
// elevation angle of each line for HS Line 40 Lidar, Line 1 - Line 40
const float hesai_elev_angle_map[] = {
    6.96, 5.976, 4.988, 3.996,
    2.999, 2.001, 1.667, 1.333,
    1.001, 0.667, 0.333, 0,
    -0.334, -0.667, -1.001, -1.334,
    -1.667, -2.001, -2.331, -2.667,
    -3, -3.327, -3.663, -3.996,
    -4.321, -4.657, -4.986, -5.311,
    -5.647, -5.974, -6.957, -7.934,
    -8.908, -9.871, -10.826, -11.772,
    -12.705, -13.63, -14.543, -15.444
};

// Line 40 Lidar azimuth Horizatal offset ,  Line 1 - Line 40
const float hesai_horizatal_azimuth_offset_map[] = {
    0.005, 0.006, 0.006, 0.006,
    -2.479, -2.479, 2.491, -4.953,
    -2.479, 2.492, -4.953, -2.479,
    2.492, -4.953, 0.007, 2.491,
    -4.953, 0.006, 4.961, -2.479,
    0.006, 4.96, -2.478, 0.006,
    4.958, -2.478, 2.488, 4.956,
    -2.477, 2.487, 2.485, 2.483,
    0.004, 0.004, 0.003, 0.003,
    -2.466, -2.463, -2.46, -2.457
};

double *pcl::PandarGrabber::cos_lookup_table_ = NULL;
double *pcl::PandarGrabber::sin_lookup_table_ = NULL;

using boost::asio::ip::udp;

/*
*   HS_L40_Parse:
*   Parse an UDP packet<recvbuf> which has <len> bytes to HS_LIDAR_L40_Packet<packet>.
*/
int HS_L40_Parse(HS_LIDAR_L40_Packet *packet , const unsigned char* recvbuf , const int len)
{
    if(len != HS_LIDAR_L40_PACKET_SIZE)
        return -1;

    int index = 0;
    int block = 0;
    // 6x BLOCKs
    for(block = 0 ; block < HS_LIDAR_L40_BLOCK_NUM ; block ++)
    {
        int unit;
        packet->blocks[block].sob = (recvbuf[index] & 0xff)| ((recvbuf[index + 1] & 0xff)<< 8);
        packet->blocks[block].Azimuth = (recvbuf[index + 2]& 0xff) | ((recvbuf[index + 3]& 0xff) << 8);
        index += HS_LIDAR_L40_SOP_ANGLE_SIZE;
        // 40x units
        for(unit = 0 ; unit < HS_LIDAR_L40_UNIT_NUM ; unit++)
        {
            packet->blocks[block].units[unit].distance = (recvbuf[index]& 0xff) | ((recvbuf[index + 1]& 0xff) << 8) | ((recvbuf[index + 2]& 0xff) << 16 );
            packet->blocks[block].units[unit].reflectivity = (recvbuf[index + 3]& 0xff) | ((recvbuf[index + 4]& 0xff) << 8);
            index += HS_LIDAR_L40_SERIAL_UNIT_SIZE;

            // TODO: Filtering wrong data for LiDAR Bugs.
            if((packet->blocks[block].units[unit].distance == 0x010101 && packet->blocks[block].units[unit].reflectivity == 0x0101) \
                    || packet->blocks[block].units[unit].distance > (200 * 1000 /2 /* 200m -> 2mm */))
            {
                // printf("filtering %x %x \n" , packet->blocks[block].units[unit].distance , packet->blocks[block].units[unit].reflectivity);
                packet->blocks[block].units[unit].distance = 0;
                packet->blocks[block].units[unit].reflectivity = 0;
            }
            //printf("reflectivity %d \n", packet->blocks[block].units[unit].reflectivity);
        }
    }

    memcpy(packet->reserved , recvbuf + index , HS_LIDAR_L40_RESERVED_SIZE);
    index += HS_LIDAR_L40_RESERVED_SIZE; // skip reserved bytes

    packet->engine_velocity = (recvbuf[index]& 0xff)| (recvbuf[index + 1]& 0xff) << 8;
    index += HS_LIDAR_L40_ENGINE_VELOCITY;
    // printf("speed %d\n", packet->engine_velocity * 6 /11);

    packet->timestamp = (recvbuf[index]& 0xff)| (recvbuf[index + 1]& 0xff) << 8 |
                        ((recvbuf[index + 2 ]& 0xff) << 16) | ((recvbuf[index + 3]& 0xff) << 24);
    // printf("timestamp %u \n", packet->timestamp);
    index += HS_LIDAR_L40_TIMESTAMP_SIZE;

    packet->factory[0] = recvbuf[index]& 0xff;
    packet->factory[1] = recvbuf[index + 1]& 0xff;
    index += HS_LIDAR_L40_FACTORY_SIZE;
    // printf("Index size should be %d  , result is : %d \n", HS_LIDAR_L40_PACKET_SIZE , index);
    return 0;
}

float HS_AzimuthWithOffset(float azimuth, int lineNum)
{
    float retAzimuth;
    if(lineNum >= HS_LIDAR_L40_UNIT_NUM)
    {
        return azimuth;
    }
    if(hesai_horizatal_azimuth_offset_map[lineNum] >= 0 )
    {
        retAzimuth = azimuth + hesai_horizatal_azimuth_offset_map[lineNum];
        return retAzimuth > 360.0 ? retAzimuth - 360.0 : retAzimuth;
    }
    else
    {
        float tempAzimuth = azimuth;
        tempAzimuth += hesai_horizatal_azimuth_offset_map[lineNum];
        if(tempAzimuth < 0)
        {
            return azimuth + 360.0 + hesai_horizatal_azimuth_offset_map[lineNum];
        }
        return tempAzimuth;
    }
}

/////////////////////////////////////////////////////////////////////////////
pcl::PandarGrabber::PandarGrabber (const std::string& correctionsFile,
                                   const std::string& pcapFile) :
    last_azimuth_ (65000),
    current_sweep_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ()),
    sweep_xyzi_signal_ (),
    Pandar_data_ (),
    udp_listener_endpoint_ (),
    source_address_filter_ (),
    source_port_filter_ (443),
    Pandar_read_socket_service_ (),
    Pandar_read_socket_ (NULL),
    pcap_file_name_ (pcapFile),
    queue_consumer_thread_ (NULL),
    Pandar_read_packet_thread_ (NULL),
    min_distance_threshold_ (0.0),
    max_distance_threshold_ (10000.0)
{
    initialize (correctionsFile);
}

/////////////////////////////////////////////////////////////////////////////
pcl::PandarGrabber::PandarGrabber (const boost::asio::ip::address& ipAddress,
                                   const unsigned short int port,
                                   const std::string& correctionsFile) :
    last_azimuth_ (65000),
    current_sweep_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ()),
    sweep_xyzi_signal_ (),
    Pandar_data_ (),
    udp_listener_endpoint_ (ipAddress, port),
    source_address_filter_ (),
    source_port_filter_ (443),
    Pandar_read_socket_service_ (),
    Pandar_read_socket_ (NULL),
    pcap_file_name_ (),
    queue_consumer_thread_ (NULL),
    Pandar_read_packet_thread_ (NULL),
    min_distance_threshold_ (0.0),
    max_distance_threshold_ (10000.0)
{
    initialize (correctionsFile);
}

/////////////////////////////////////////////////////////////////////////////
pcl::PandarGrabber::~PandarGrabber () throw ()
{
    stop ();
    disconnect_all_slots<sig_cb_Hesai_Pandar_sweep_point_cloud_xyzi> ();
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::PandarGrabber::initialize (const std::string& correctionsFile)
{
    // std::cout<<correctionsFile<<std::endl;
    if (cos_lookup_table_ == NULL && sin_lookup_table_ == NULL)
    {
        cos_lookup_table_ = static_cast<double *> (malloc (Pandar_NUM_ROT_ANGLES * sizeof (*cos_lookup_table_)));
        sin_lookup_table_ = static_cast<double *> (malloc (Pandar_NUM_ROT_ANGLES * sizeof (*sin_lookup_table_)));
        for (int i = 0; i < Pandar_NUM_ROT_ANGLES; i++)
        {
            double rad = (M_PI / 180.0) * (static_cast<double> (i) / 100.0);
            cos_lookup_table_[i] = std::cos (rad);
            sin_lookup_table_[i] = std::sin (rad);
        }
    }

    loadCorrectionsFile (correctionsFile);

    for (int i = 0; i < Pandar_MAX_NUM_LASERS; i++)
    {
        PandarLaserCorrection correction = laser_corrections_[i];
        laser_corrections_[i].sinVertOffsetCorrection = correction.verticalOffsetCorrection * correction.sinVertCorrection;
        laser_corrections_[i].cosVertOffsetCorrection = correction.verticalOffsetCorrection * correction.cosVertCorrection;
    }
    sweep_xyzi_signal_ = createSignal<sig_cb_Hesai_Pandar_sweep_point_cloud_xyzi> ();
    current_sweep_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI>);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::PandarGrabber::loadCorrectionsFile (const std::string& correctionsFile)
{
    loadPandar40DefaultCorrections ();
    if (correctionsFile.empty ())
        return;

    boost::filesystem::path file_path(correctionsFile);
    if (!boost::filesystem::is_regular(file_path)) {
        file_path = boost::filesystem::path("pandar40.csv");
        if (!boost::filesystem::is_regular(file_path))
            return;
    }

    std::ifstream ifs(file_path.string());
    if (!ifs.is_open())
        return;

    std::string line;
    if (std::getline(ifs, line)) {	// first line "Laser id,Elevation,Azimuth"
        std::cout << "parsing correction file." << std::endl;
    }

    int line_cnt = 0;
    while (std::getline(ifs, line)) {
        if (line_cnt++ >= 40)
            break;

        int line_id = 0;
        double elev, azimuth;

        std::stringstream ss(line);
        std::string subline;
        std::getline(ss, subline, ',');
        std::stringstream(subline) >> line_id;
        std::getline(ss, subline, ',');
        std::stringstream(subline) >> elev;
        std::getline(ss, subline, ',');
        std::stringstream(subline) >> azimuth;
        line_id--;
        laser_corrections_[line_id].azimuthCorrection = azimuth;
        laser_corrections_[line_id].distanceCorrection = 0.0;
        laser_corrections_[line_id].horizontalOffsetCorrection = 0.0;
        laser_corrections_[line_id].verticalOffsetCorrection = 0.0;
        laser_corrections_[line_id].verticalCorrection = elev;
        laser_corrections_[line_id].sinVertCorrection = std::sin (Pandar_Grabber_toRadians(elev));
        laser_corrections_[line_id].cosVertCorrection = std::cos (Pandar_Grabber_toRadians(elev));
    }
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::PandarGrabber::loadPandar40DefaultCorrections ()
{
    for (int i = 0; i < Pandar_LASER_PER_FIRING; i++)
    {
        laser_corrections_[i].azimuthCorrection = hesai_horizatal_azimuth_offset_map[i];
        laser_corrections_[i].distanceCorrection = 0.0;
        laser_corrections_[i].horizontalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalCorrection = hesai_elev_angle_map[i];
        laser_corrections_[i].sinVertCorrection = std::sin (Pandar_Grabber_toRadians(hesai_elev_angle_map[i]));
        laser_corrections_[i].cosVertCorrection = std::cos (Pandar_Grabber_toRadians(hesai_elev_angle_map[i]));
    }
}

/////////////////////////////////////////////////////////////////////////////
boost::asio::ip::address
pcl::PandarGrabber::getDefaultNetworkAddress ()
{
    return (boost::asio::ip::address::from_string ("192.168.3.255"));
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::PandarGrabber::processHesaiPackets ()
{
    while (true)
    {
        // std::cout<<"Process Data"<<std::endl;
        unsigned char *data;
        if (!Pandar_data_.dequeue (data))
            return;

        HS_LIDAR_L40_Packet *packet = (HS_LIDAR_L40_Packet* )malloc(sizeof(HS_LIDAR_L40_Packet));
        int ret = HS_L40_Parse(packet , data , HS_LIDAR_L40_PACKET_SIZE);
        if(ret)
            return;

        // std::cout<<"angle : " << packet->blocks[0].Azimuth <<std::endl;

        toPointClouds (packet);

        free(packet);
        free (data);
    }
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::PandarGrabber::toPointClouds (HS_LIDAR_L40_Packet *dataPacket)
{
    static uint32_t sweep_counter = 0;

    time_t system_time;
    time (&system_time);
    time_t Hesai_time = (system_time & 0x00000000ffffffffl) << 32 | dataPacket->timestamp;

    for (int i = 0; i < HS_LIDAR_L40_BLOCK_NUM; ++i)
    {
        HS_LIDAR_L40_Block firing_data = dataPacket->blocks[i];

        for (int j = 0; j < HS_LIDAR_L40_UNIT_NUM; j++)
        {
            if (firing_data.Azimuth < last_azimuth_)
            {
                if (current_sweep_xyzi_->size () > 0)
                {
                    current_sweep_xyzi_->header.stamp = Hesai_time;
                    current_sweep_xyzi_->header.seq = sweep_counter;
                    sweep_counter++;
                    fireCurrentSweep ();
                }
                current_sweep_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI> ());
            }

            PointXYZI xyzi;
            computeXYZI (xyzi, firing_data.Azimuth, firing_data.units[j], laser_corrections_[j]);

            if (pcl_isnan (xyzi.x) || pcl_isnan (xyzi.y) || pcl_isnan (xyzi.z))
            {
                continue;
            }

            current_sweep_xyzi_->push_back (xyzi);
            last_azimuth_ = firing_data.Azimuth;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::PandarGrabber::computeXYZI (pcl::PointXYZI& point,
                                 int azimuth,
                                 HS_LIDAR_L40_Unit laserReturn,
                                 PandarLaserCorrection correction)
{
    double cos_azimuth, sin_azimuth;

    double distanceM = laserReturn.distance * 0.002;

    point.intensity = static_cast<float> (laserReturn.reflectivity >> 8);
    if (distanceM < min_distance_threshold_ || distanceM > max_distance_threshold_)
    {
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
        return;
    }
    if (correction.azimuthCorrection == 0)
    {
        cos_azimuth = cos_lookup_table_[azimuth];
        sin_azimuth = sin_lookup_table_[azimuth];
    }
    else
    {
        // std::cout<<"2"<<correction.azimuthCorrection<<std::endl;
        double azimuthInRadians = Pandar_Grabber_toRadians( (static_cast<double> (azimuth) / 100.0) + correction.azimuthCorrection);
        cos_azimuth = std::cos (azimuthInRadians);
        sin_azimuth = std::sin (azimuthInRadians);
    }

    distanceM += correction.distanceCorrection;

    double xyDistance = distanceM * correction.cosVertCorrection;

    point.x = static_cast<float> (xyDistance * sin_azimuth - correction.horizontalOffsetCorrection * cos_azimuth);
    point.y = static_cast<float> (xyDistance * cos_azimuth + correction.horizontalOffsetCorrection * sin_azimuth);
    point.z = static_cast<float> (distanceM * correction.sinVertCorrection + correction.verticalOffsetCorrection);
    if (point.x == 0 && point.y == 0 && point.z == 0)
    {
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
    }
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::PandarGrabber::fireCurrentSweep ()
{
    if (sweep_xyzi_signal_ != NULL && sweep_xyzi_signal_->num_slots () > 0)
        sweep_xyzi_signal_->operator() (current_sweep_xyzi_);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::PandarGrabber::enqueuePandarPacket (const unsigned char *data,
        std::size_t bytesReceived)
{
    if (bytesReceived == HS_LIDAR_L40_PACKET_SIZE)
    {
        unsigned char *dup = static_cast<unsigned char *> (malloc (bytesReceived * sizeof(unsigned char)));
        memcpy (dup, data, bytesReceived * sizeof(unsigned char));

        Pandar_data_.enqueue (dup);
    }
	if (Pandar_data_.size() > 128) {
		unsigned char* data;
		Pandar_data_.dequeue(data);
		free(data);
	}
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::PandarGrabber::start ()
{
    terminate_read_packet_thread_ = false;

    if (isRunning ())
        return;

    queue_consumer_thread_ = new boost::thread (boost::bind (&PandarGrabber::processHesaiPackets, this));
    // std::cout<<pcap_file_name_<<std::endl;
    if (pcap_file_name_.empty ()) {
        try {
            try {
                if (isAddressUnspecified (udp_listener_endpoint_.address ())) {
                    udp_listener_endpoint_.address (getDefaultNetworkAddress ());
                }
                if (udp_listener_endpoint_.port () == 0) {
                    udp_listener_endpoint_.port (Pandar_DATA_PORT);
                }
                Pandar_read_socket_ = new udp::socket (Pandar_read_socket_service_, udp_listener_endpoint_);
            } catch (const std::exception& bind) {
                delete Pandar_read_socket_;
                Pandar_read_socket_ = new udp::socket (Pandar_read_socket_service_, udp::endpoint (boost::asio::ip::address_v4::any (), udp_listener_endpoint_.port ()));
            }
            Pandar_read_socket_service_.run ();
        } catch (std::exception &e) {
            PCL_ERROR("[pcl::PandarGrabber::start] Unable to bind to socket! %s\n", e.what ());
            return;
        }
        Pandar_read_packet_thread_ = new boost::thread (boost::bind (&PandarGrabber::readPacketsFromSocket, this));
    } else {
        Pandar_read_packet_thread_ = new boost::thread(boost::bind(&PandarGrabber::readPacketsFromPcap, this));
    }
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::PandarGrabber::stop ()
{
    terminate_read_packet_thread_ = true;
    Pandar_data_.stopQueue ();

    if (Pandar_read_packet_thread_ != NULL)
    {
        Pandar_read_packet_thread_->interrupt ();
        Pandar_read_packet_thread_->join ();
        delete Pandar_read_packet_thread_;
        Pandar_read_packet_thread_ = NULL;
    }
    if (queue_consumer_thread_ != NULL)
    {
        queue_consumer_thread_->join ();
        delete queue_consumer_thread_;
        queue_consumer_thread_ = NULL;
    }

    if (Pandar_read_socket_ != NULL)
    {
        delete Pandar_read_socket_;
        Pandar_read_socket_ = NULL;
    }
}

/////////////////////////////////////////////////////////////////////////////
bool
pcl::PandarGrabber::isRunning () const
{
    return (!Pandar_data_.isEmpty () || (Pandar_read_packet_thread_ != NULL && !Pandar_read_packet_thread_->timed_join (boost::posix_time::milliseconds (10))));
}

/////////////////////////////////////////////////////////////////////////////
std::string
pcl::PandarGrabber::getName () const
{
    return (std::string ("Hesai LiDAR (Pandar) Grabber"));
}

/////////////////////////////////////////////////////////////////////////////
float
pcl::PandarGrabber::getFramesPerSecond () const
{
    return (0.0f);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::PandarGrabber::filterPackets (const boost::asio::ip::address& ipAddress,
                                   const unsigned short port)
{
    source_address_filter_ = ipAddress;
    source_port_filter_ = port;
}

/////////////////////////////////////////////////////////////////////////////
bool
pcl::PandarGrabber::isAddressUnspecified (const boost::asio::ip::address& ipAddress)
{
#if BOOST_VERSION>=104700
    return (ipAddress.is_unspecified ());
#else
    if (ipAddress.is_v4 ())
        return (ipAddress.to_v4 ().to_ulong() == 0);

    return (false);
#endif
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::PandarGrabber::setMaximumDistanceThreshold (float &maxThreshold)
{
    max_distance_threshold_ = maxThreshold;
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::PandarGrabber::setMinimumDistanceThreshold (float &minThreshold)
{
    min_distance_threshold_ = minThreshold;
}

/////////////////////////////////////////////////////////////////////////////
float
pcl::PandarGrabber::getMaximumDistanceThreshold ()
{
    return (max_distance_threshold_);
}

/////////////////////////////////////////////////////////////////////////////
float
pcl::PandarGrabber::getMinimumDistanceThreshold ()
{
    return (min_distance_threshold_);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::PandarGrabber::readPacketsFromSocket ()
{
    unsigned char data[1500];
    udp::endpoint sender_endpoint;

    while (!terminate_read_packet_thread_ && Pandar_read_socket_->is_open ())
    {
        size_t length = Pandar_read_socket_->receive_from (boost::asio::buffer (data, 1500), sender_endpoint);

        if (isAddressUnspecified (source_address_filter_)
                || (source_address_filter_ == sender_endpoint.address () && source_port_filter_ == sender_endpoint.port ()))
        {
            enqueuePandarPacket (data, length);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////

void
pcl::PandarGrabber::readPacketsFromPcap ()
{
    struct pcap_pkthdr *header;
    const unsigned char *data;
    char errbuff[PCAP_ERRBUF_SIZE];

    pcap_t *pcap = pcap_open_offline (pcap_file_name_.c_str (), errbuff);

    struct bpf_program filter;
    std::ostringstream string_stream;

    string_stream << "udp ";
    if (!isAddressUnspecified(source_address_filter_))
    {
        string_stream << " and src port " << source_port_filter_ << " and src host " << source_address_filter_.to_string();
    }

    // PCAP_NETMASK_UNKNOWN should be 0xffffffff, but it's undefined in older PCAP versions
    if (pcap_compile (pcap, &filter, string_stream.str ().c_str(), 0, 0xffffffff) == -1)
    {
        PCL_WARN ("[pcl::PandarGrabber::readPacketsFromPcap] Issue compiling filter: %s.\n", pcap_geterr (pcap));
    }
    else if (pcap_setfilter(pcap, &filter) == -1)
    {
        PCL_WARN ("[pcl::PandarGrabber::readPacketsFromPcap] Issue setting filter: %s.\n", pcap_geterr (pcap));
    }

    struct timeval lasttime;
    unsigned long long usec_delay;

    lasttime.tv_sec = 0;

    int returnValue = pcap_next_ex(pcap, &header, &data);

    while (returnValue >= 0 && !terminate_read_packet_thread_)
    {
        // std::cout<<"process"<<std::endl;
        if (lasttime.tv_sec == 0)
        {
            lasttime.tv_sec = header->ts.tv_sec;
            lasttime.tv_usec = header->ts.tv_usec;
        }
        if (lasttime.tv_usec > header->ts.tv_usec)
        {
            lasttime.tv_usec -= 1000000;
            lasttime.tv_sec++;
        }
        usec_delay = ((header->ts.tv_sec - lasttime.tv_sec) * 1000000) +
                     (header->ts.tv_usec - lasttime.tv_usec);

        boost::this_thread::sleep(boost::posix_time::microseconds(usec_delay));

        lasttime.tv_sec = header->ts.tv_sec;
        lasttime.tv_usec = header->ts.tv_usec;

        // The ETHERNET header is 42 bytes long; unnecessary
        enqueuePandarPacket(data + 42, header->len - 42);

        returnValue = pcap_next_ex(pcap, &header, &data);
    }
}
