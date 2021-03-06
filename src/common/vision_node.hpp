/*********************************************************************************
 *
 *  Copyright (c) 2014, Donato Di Paola
 *
 *  Software License Agreement (MIT License)
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 *********************************************************************************/

#ifndef _VISION_NODE_HPP_
#define _VISION_NODE_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

class VisionNode {	
	protected:		
		ros::NodeHandle nh_;
	
		// ROS image_transport declaretions			
		image_transport::ImageTransport it_;
		image_transport::CameraSubscriber m_camera_sub_;
		image_geometry::PinholeCameraModel m_cam_model_;
					
		// OpenCV image declareations
		cv::Mat m_img_src;
		cv::Size m_img_src_size;

		cv::Mat m_img_work;
		double m_img_work_scale;

		virtual void runVisionCore()=0;
		virtual void publishResults()=0;

	public:
		VisionNode();
		~VisionNode();

		void visionCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
	 
};


#endif // _VISION_NODE_HPP_
