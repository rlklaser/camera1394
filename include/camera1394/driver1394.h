/* -*- mode: C++ -*- */
/* $Id$ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010 Jack O'Quin
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
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
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
*********************************************************************/

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <driver_base/driver.h>
#include <dynamic_reconfigure/server.h>

#include <camera1394/dev_camera1394.h>
#include <camera1394/Camera1394Config.h>
typedef camera1394::Camera1394Config Config;

/** @file

    @brief ROS driver interface for IIDC-compatible IEEE 1394 digital cameras.

*/

namespace camera1394_driver
{

class Camera1394Driver
{
public:

  // public methods
  Camera1394Driver(ros::NodeHandle priv_nh,
                   ros::NodeHandle camera_nh);
  virtual ~Camera1394Driver();
  void poll(void);
  void setup(void);
  void shutdown(void);

private:

  // private methods
  void closeCamera();
  bool openCamera(Config &newconfig);
  bool read(sensor_msgs::ImagePtr &image);
  void reconfig(camera1394::Camera1394Config &newconfig, uint32_t level);

protected:

  //protected methods
  virtual void publish(const sensor_msgs::ImagePtr &image) = 0;
  virtual inline void newCameraName() {};
  virtual bool validateConfig(Config &newconfig) = 0;

  /** Non-recursive mutex for serializing callbacks with device polling. */
  boost::mutex mutex_;

  /** driver state variables */
  volatile driver_base::Driver::state_t state_; // current driver state
  volatile bool reconfiguring_;         // true when reconfig() running
  ros::NodeHandle priv_nh_;             // private node handle
  ros::NodeHandle camera_nh_;           // camera name space handle
  std::string camera_name_;             // camera name
  ros::Rate cycle_;                     // polling rate when closed
  uint32_t retries_;                    // count of openCamera() retries

  /** libdc1394 camera device interface */
  boost::shared_ptr<camera1394::Camera1394> dev_;

  /** dynamic parameter configuration */
  camera1394::Camera1394Config config_;
  dynamic_reconfigure::Server<camera1394::Camera1394Config> srv_;

  /** diagnostics updater */
  diagnostic_updater::Updater diagnostics_;
  double topic_diagnostics_min_freq_;
  double topic_diagnostics_max_freq_;
  diagnostic_updater::TopicDiagnostic topic_diagnostics_;

}; // end class Camera1394Driver

}; // end namespace camera1394_driver