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

#include "vision_node.hpp"


VisionNode::VisionNode() : it_(nh_){
	// Set the subscriber
	m_camera_sub_ = it_.subscribeCamera(nh_.resolveName("image"), 1, &VisionNode::visionCallback, this, image_transport::TransportHints("raw"));
	//m_img_work_scale = 0.5;
}

VisionNode::~VisionNode(){
}

void VisionNode::visionCallback(const sensor_msgs::ImageConstPtr& image_msg,const sensor_msgs::CameraInfoConstPtr& info_msg){

	// Convert the image to OpenCV format
	cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }  

	// Get the calibration info from 'camera_info'
	m_cam_model_.fromCameraInfo(info_msg);

	// Set the source image
	m_img_src = cv_ptr->image;
	m_img_src_size = m_img_src.size();
	cv::Size workingSize(m_img_src_size.width*m_img_work_scale,m_img_src_size.height*m_img_work_scale);
	
	// Copy the source image to the work image with a working size	    
	cv::resize(m_img_src, m_img_work, workingSize);

	/////////////////////////		  
	// Vision Processing Core
	runVisionCore();

	/////////////////////////

	// Resize the work image to the source image size	    
	//cv::resize(m_img_work, m_img_src, m_img_src_size);

	// Publish results
	publishResults();
}


