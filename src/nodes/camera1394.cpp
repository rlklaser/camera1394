/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2013, Laboratorio de Robotica Movel - ICMC/USP
*  Rafael Luiz Klaser <rlklaser@gmail.com>
*  http://lrm.icmc.usp.br
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
 * @file camera1394.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Aug 1, 2013
 *
 */

#include "camera1394/camera1394.h"

namespace camera1394_driver
{
Camera1394::Camera1394(ros::NodeHandle priv_nh, ros::NodeHandle camera_nh) :
		Camera1394Driver(priv_nh, camera_nh),
		cinfo_(new camera_info_manager::CameraInfoManager(camera_nh_)),
		calibration_matches_(true),
		it_(new image_transport::ImageTransport(camera_nh_)),
		image_pub_(it_->advertiseCamera("image_raw", 1))
{
}

void Camera1394::newCameraName()
{
	if (!cinfo_->setCameraName(camera_name_))
	{
		// GUID is 16 hex digits, which should be valid.
		// If not, use it for log messages anyway.
		ROS_WARN_STREAM("[" << camera_name_ << "] name not valid" << " for camera_info_manger");
	}
}

bool Camera1394::validateConfig(Config &newconfig)
{
	bool is_valid = false;
    // set the new URL and load CameraInfo (if any) from it
	if (cinfo_->validateURL(newconfig.camera_info_url))
	{
		cinfo_->loadCameraInfo(newconfig.camera_info_url);
		is_valid = true;
	}

	return is_valid;
}
/** Publish camera stream topics
 *
 *  @param image points to latest camera frame
 */
void Camera1394::publish(const sensor_msgs::ImagePtr &image)
{
	image->header.frame_id = config_.frame_id;

	// get current CameraInfo data
	sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));

	// check whether CameraInfo matches current video mode
	if (!dev_->checkCameraInfo(*image, *ci))
	{
		// image size does not match: publish a matching uncalibrated
		// CameraInfo instead
		if (calibration_matches_)
		{
			// warn user once
			calibration_matches_ = false;
			ROS_WARN_STREAM("[" << camera_name_ << "] calibration does not match video mode " << "(publishing uncalibrated data)");
		}
		ci.reset(new sensor_msgs::CameraInfo());
		ci->height = image->height;
		ci->width = image->width;
	}
	else if (!calibration_matches_)
	{
		// calibration OK now
		calibration_matches_ = true;
		ROS_WARN_STREAM("[" << camera_name_ << "] calibration matches video mode now");
	}

	// fill in operational parameters
	dev_->setOperationalParameters(*ci);

	ci->header.frame_id = config_.frame_id;
	ci->header.stamp = image->header.stamp;

	// Publish via image_transport
	image_pub_.publish(image, ci);

	// Notify diagnostics that a message has been published. That will
	// generate a warning if messages are not published at nearly the
	// configured frame_rate.
	topic_diagnostics_.tick(image->header.stamp);
}
}

