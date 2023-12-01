/* includes etc. //{ */

#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <std_srvs/Trigger.h>

#include <cmath>
#include <mutex>
#include <thread>

#include <nodelet/nodelet.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <ouster_ros/GetMetadata.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>

#include "vofod/types.h"

using namespace cv;
using namespace std;
using namespace vofod;
using namespace pcl;

//}

namespace vofod
{
  class MaskCreator : public nodelet::Nodelet
  {
  public:
    /* onInit() method //{ */
    void onInit()
    {
      ros::NodeHandle nh = nodelet::Nodelet::getPrivateNodeHandle();
      ROS_INFO("[MaskCreator]: Waiting for valid time...");
      ros::Time::waitForValid();

      m_node_name = "MaskCreator";

      /* Load parameters from ROS //{*/
      mrs_lib::ParamLoader pl(nh, m_node_name);
      // LOAD STATIC PARAMETERS
      NODELET_INFO("Loading static parameters:");
      const auto uav_name = pl.loadParam2<std::string>("uav_name");
      pl.loadParam("sensor/simulation", m_sensor_simulation);
      pl.loadParam("use_opencv", m_use_opencv);
      pl.loadParam("mask_filename", m_mask_fname);

      // CHECK LOADING STATUS
      if (!pl.loadedSuccessfully())
      {
        NODELET_ERROR("Some compulsory parameters were not loaded successfully, ending the node");
        ros::shutdown();
        return;
      }
      //}

      /* Create publishers and subscribers //{ */

      // Initialize subscribers
      mrs_lib::SubscribeHandlerOptions shopts(nh);
      shopts.no_message_timeout = ros::Duration(5.0);

      mrs_lib::construct_object(m_sh_pc, shopts, "pointcloud", &MaskCreator::cloud_callback, this);

      // Initialize publishers
      image_transport::ImageTransport it(nh);
      m_pub_mask = it.advertise("mask", 1);

      m_reset_server = nh.advertiseService("reset", &MaskCreator::reset_callback, this);
      m_save_server = nh.advertiseService("save", &MaskCreator::save_callback, this);
      //}

      reset();

      // initialize the sensor information
      m_sensor_initialized = false;
      std::thread sensor_load_thread(&MaskCreator::initialize_sensor, this);
      sensor_load_thread.detach();

      m_disp_thread = std::thread(&MaskCreator::display_loop, this);

      std::cout << "----------------------------------------------------------" << std::endl;
    }
    //}

    /* initialize_sensor() method //{ */
    void initialize_sensor_rosparam()
    {
      ros::NodeHandle nh = nodelet::Nodelet::getPrivateNodeHandle();
      mrs_lib::ParamLoader pl(nh, m_node_name);
      pl.loadParam("sensor/vertical_fov_angle", m_sensor_vfov);
      pl.loadParam("sensor/vertical_rays", m_sensor_vrays);
      pl.loadParam("sensor/horizontal_rays", m_sensor_hrays);

      // CHECK LOADING STATUS
      if (pl.loadedSuccessfully())
      {
        m_sensor_initialized = true;
      }
      else
      {
        NODELET_ERROR("Could not load sensor parameters from rosparam server, ending the node");
        ros::shutdown();
      }
    }

    void initialize_sensor()
    {
      ros::NodeHandle nh = nodelet::Nodelet::getPrivateNodeHandle();
      auto client = nh.serviceClient<ouster_ros::GetMetadata>("get_metadata");
      std::vector<int> pixel_shift_by_row;

      NODELET_INFO_STREAM("[PCLDetector]: Waiting 15s for service \"" << client.getService() << "\" to become available.");
      if (m_sensor_simulation || !client.waitForExistence(ros::Duration(25.0)))
      {
        if (m_sensor_simulation)
          NODELET_ERROR("[PCLDetector]: Using a simulated sensor! Loading data from rosparam server.");
        else
          NODELET_ERROR("[PCLDetector]: OS config service is not ready in 10s! Loading data from rosparam server (DATA WILL BE UNCALIBRATED!!).");
        initialize_sensor_rosparam();
        pixel_shift_by_row.resize(m_sensor_hrays, 0);
      }
      else
      {
        ouster_ros::GetMetadata cfg;
        if (!client.call(cfg))
        {
          NODELET_ERROR("[PCLDetector]: Calling OS config service failed! Loading data from rosparam server (DATA WILL BE UNCALIBRATED!!).");
          initialize_sensor_rosparam();
          pixel_shift_by_row.resize(m_sensor_hrays, 0);
        }
        else
        {
          const auto info = ouster::sensor::parse_metadata(cfg.response.metadata);
          const auto H = info.format.pixels_per_column;
          const auto W = info.format.columns_per_frame;
          pixel_shift_by_row = info.format.pixel_shift_by_row;

          NODELET_INFO("[PCLDetector]: Calling OS config service succeeded! Initializing sensor parameters from the received response.");
          m_sensor_vrays = H;
          m_sensor_hrays = W;
          m_sensor_vfov = std::abs(info.beam_altitude_angles.back() - info.beam_altitude_angles.front());
        }
      }
      
      // Load the mask and print some info to the console
      NODELET_INFO_STREAM("[PCLDetector]: Initialized using sensor parameters:" << std::endl
          << "\tvertical rays: " << m_sensor_vrays
          << "\tvertical FOV: " << m_sensor_vfov
          << "\thorizontal rays: " << m_sensor_hrays
          );
      std::scoped_lock lck(m_mask_mtx);
      m_sensor_mask_img = cv::Mat(cv::Size(m_sensor_hrays, m_sensor_vrays), CV_8UC1, 255);
      m_sensor_initialized = true;
    }
    //}

    void display_loop()
    {
      while (ros::ok())
      {
        if (!m_sensor_initialized || m_sensor_mask_img.empty())
          continue;
        cv::Mat img_copy;
        {
          std::scoped_lock lck(m_mask_mtx);
          m_sensor_mask_img.copyTo(img_copy);
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_copy).toImageMsg();
        m_pub_mask.publish(msg);

        if (m_use_opencv)
        {
          cv::imshow("mask", img_copy);
          const int key = cv::waitKey(10);
          if (key == 's')
            save();
        }

        ros::Duration(0.05).sleep();
      }
    }

    /* reset_callback() method //{ */

    bool reset_callback([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
    {
      reset();
      resp.message = "Mask reset.";
      resp.success = true;
      return true;
    }

    //}

    /* save_callback() method //{ */

    bool save_callback([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
    {
      save();
      resp.message = "Mask saved.";
      resp.success = true;
      return true;
    }

    //}

  private:
    /* cloud_callback() method //{ */
    void cloud_callback(pc_t::ConstPtr cloud)
    {
      if (!m_sensor_initialized)
        return;

      NODELET_INFO_STREAM_THROTTLE(1.0, "Received cloud message.");
      std::scoped_lock lck(m_mask_mtx);

      for (int row = 0; row < (int)cloud->height; row++)
      {
        for (int col = 0; col < (int)cloud->width; col++)
        {
          const auto pt = cloud->at(col, row);
          const int idx = int(row * cloud->width + col);
          if (pt.range == 0)
            m_sensor_mask_img.at<uint8_t>(idx) = 0;
        }
      }
    }
    //}

    /* reset() method //{ */

    void reset()
    {
      std::scoped_lock lck(m_mask_mtx);

      m_sensor_mask_img.setTo(255);

      NODELET_WARN_THROTTLE(1.0, "[MaskCreator]: Mask reset!");
    }

    //}

    /* save() method //{ */

    void save()
    {
      std::scoped_lock lck(m_mask_mtx);

      cv::imwrite(m_mask_fname, m_sensor_mask_img);

      NODELET_WARN_THROTTLE(1.0, "[MaskCreator]: Mask saved!");
    }

    //}

  private:
    // --------------------------------------------------------------
    // |                ROS-related member variables                |
    // --------------------------------------------------------------

    /* ROS related variables (subscribers, timers etc.) //{ */
    mrs_lib::SubscribeHandler<pc_t> m_sh_pc;

    image_transport::Publisher m_pub_mask;

    ros::ServiceServer m_reset_server;
    ros::ServiceServer m_save_server;

    std::string m_node_name;
    //}

  private:
    // --------------------------------------------------------------
    // |                 Parameters, loaded from ROS                |
    // --------------------------------------------------------------

    bool m_use_opencv;
    std::string m_mask_fname;

  private:
    // --------------------------------------------------------------
    // |                  Sensor-related variables                  |
    // --------------------------------------------------------------

    bool m_sensor_initialized;
    bool m_sensor_simulation;

    std::thread m_disp_thread;

    std::mutex m_mask_mtx;
    std::vector<uint8_t> m_sensor_mask;
    cv::Mat m_sensor_mask_img;
    float m_sensor_vfov;
    int m_sensor_vrays;
    int m_sensor_hrays;

  };  // class PCLDetector
};    // namespace vofod

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vofod::MaskCreator, nodelet::Nodelet)

