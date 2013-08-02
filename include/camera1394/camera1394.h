/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2013, Laboratorio de Robotica Movel - ICMC/USP
*  Rafael Luiz Klaser <rlklaser@gmail.com>
*  http://lrm.icmc.usp.br
*
*  Apoio FAPESP: 2012/04555-4
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

/**
 * @file camera1394.h
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Aug 1, 2013
 *
 */

#ifndef CAMERA1394_H_
#define CAMERA1394_H_

#include <ros/ros.h>

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

#include <camera1394/driver1394.h>

namespace camera1394_driver
{

class Camera1394 : public Camera1394Driver
{
public:
	Camera1394(ros::NodeHandle priv_nh, ros::NodeHandle camera_nh);
	//virtual Camera1394();

protected:
	virtual void publish(const sensor_msgs::ImagePtr &image);
	virtual void newCameraName();
	virtual bool validateConfig(Config &newconfig);

private:
	/** camera calibration information */
	boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
	bool calibration_matches_;            // CameraInfo matches video mode

	/** image transport interfaces */
	boost::shared_ptr<image_transport::ImageTransport> it_;
	image_transport::CameraPublisher image_pub_;
};

};

#endif /* CAMERA1394_H_ */
