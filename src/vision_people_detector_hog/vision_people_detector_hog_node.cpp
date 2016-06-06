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

#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <people_detector/vision_people_detector_hogConfig.h>

#include <ecl/linear_algebra.hpp>
#include <geometry_msgs/PointStamped.h>

#include "vision_people_detector_hog.hpp"
#include "vision_people_detector_hog_node.hpp"



PeopleDetectorHOGNode::PeopleDetectorHOGNode() : VisionNode(){

	// Set the publishers
	m_image_pub_ = it_.advertise(ros::this_node::getName() + "/image_labeled", 1);
	m_positioncov_pub_ = nh_.advertise<people_detector::PointWithCovarianceStamped>(ros::this_node::getName() + "/person_pointcov",1);	
	m_position_pub_ = nh_.advertise<geometry_msgs::PointStamped>(ros::this_node::getName() + "/person_point",1);
}

PeopleDetectorHOGNode::~PeopleDetectorHOGNode(){
}

void PeopleDetectorHOGNode::configCallback(people_detector::vision_people_detector_hogConfig &config, uint32_t level){
	// Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
	m_img_work_scale = config.image_scale;
	m_hog_win_stride = config.hog_win_stride;
	m_hog_scale0 = config.hog_scale0;
} 

void PeopleDetectorHOGNode::runVisionCore(){
	
	PeopleDetectorHOG pdHog(m_hog_win_stride, m_hog_scale0);

	// Run the People Detector
	pdHog.detectObjects(m_img_work);

	// Update the node objects
	m_peopleBoxes = pdHog.getObjBoxes();
	m_peopleWeights = pdHog.getObjWeights();			
	

	// Get the 2D person position on the image 
	cv::vector<cv::Point2d> peoplePositions2d = pdHog.getPeoplePositions();	

	// Project the person position onto the world plane
	m_peoplePositionsOnMap.clear();
	
	for (size_t i=0; i<peoplePositions2d.size(); i++){
		people_detector::PointWithCovarianceStamped personPositionOnMap = getMapCoords(peoplePositions2d[i], ros::Time::now());
		m_peoplePositionsOnMap.push_back(personPositionOnMap);
  	}

	// Get the image labeled with valid boxes
	m_img_src = getOutputImage();	

}


cv::Mat PeopleDetectorHOGNode::getOutputImage(){

	cv::Mat img = m_img_src;

	// Draw boxes on the image
	for (size_t i=0; i<m_peopleBoxes.size(); i++){
		cv::Point2d tl_point = m_peopleBoxes[i].tl();
		cv::Point2d br_point = m_peopleBoxes[i].br();	
		
		tl_point.x /= m_img_work_scale;
		tl_point.y /= m_img_work_scale;
		br_point.x /= m_img_work_scale;
		br_point.y /= m_img_work_scale;
		cv::rectangle(img, tl_point, br_point, cv::Scalar(0,255,0), 2);

		// weights are written in the bottom right corner of the detection box
		//char numStr[80];
		//sprintf(numStr,"%f",m_peopleWeights[i]);
		//putText(img, numStr, br_point, CV_FONT_NORMAL, 0.6, cv::Scalar(0,255,0), 1, 1, false);	
	}

	return img;
}

people_detector::PointWithCovarianceStamped PeopleDetectorHOGNode::getMapCoords(cv::Point2d point_img, ros::Time stamp){
	
	people_detector::PointWithCovarianceStamped point_map;

  point_img.x /= m_img_work_scale;
  point_img.y /= m_img_work_scale;


  cv::Point3d point_ray = m_cam_model_.projectPixelTo3dRay(point_img);
	geometry_msgs::Vector3Stamped img_vec, img_vec_trans;

	img_vec.header.frame_id = m_cam_model_.tfFrame();
  img_vec.header.stamp = ros::Time(0);
  img_vec.vector.x = point_ray.x; img_vec.vector.y = point_ray.y; img_vec.vector.z = point_ray.z; 

  tf::StampedTransform transform;
  try{
      ros::Time acquisition_time = ros::Time(0);
      ros::Duration timeout(1.0 / 30);
			m_tf_listener.transformVector("/map", img_vec, img_vec_trans);
      m_tf_listener.lookupTransform("/map", m_cam_model_.tfFrame(), acquisition_time, transform);
  }
  catch (tf::TransformException& ex){
      ROS_WARN("[vision_people_detector_hog_node] exception:\n%s", ex.what());
      return point_map;
  }


	// p = p0 + t * v, pz = 0
  double t = - transform.getOrigin().z() / img_vec_trans.vector.z;

  // find the position on the floor
  double floor_pos_x = transform.getOrigin().x()+t*img_vec_trans.vector.x;
  double floor_pos_y = transform.getOrigin().y()+t*img_vec_trans.vector.y;
	
	// build covariance matrix
 	double d_i = sqrt( pow(floor_pos_x - transform.getOrigin().x(),2) + pow(floor_pos_y - transform.getOrigin().y(),2) + pow(transform.getOrigin().z(),2) );
	double theta_i = atan2( floor_pos_y - transform.getOrigin().y(), floor_pos_x - transform.getOrigin().x());

	double a = 0.0092, b = 0.3258;
	double k_theta = 0.1, r_sensing= 13;
	
	double sigma_d = a * exp(b*d_i);
	double sigma_theta = k_theta*d_i/r_sensing;

	ecl::linear_algebra::Matrix2d B, R, T;

	// Range-bearing covariance matrix
	B << 	sigma_d, 0, 
				0, sigma_theta;
	// Rotation matrix
	T <<	cos(theta_i), -sin(theta_i),
				sin(theta_i), cos(theta_i);
	// Cartesian covariance matrix
	R = T*B*T.transpose();
  
	// set the point on the map with covariance
	point_map.header.frame_id = "/map";
  point_map.header.stamp = stamp;
  point_map.point.point.x = floor_pos_x;
  point_map.point.point.y = floor_pos_y;
	point_map.point.covariance[0] = R(0,0);
	point_map.point.covariance[1] = R(0,1);
	point_map.point.covariance[2] = R(1,0);
	point_map.point.covariance[3] = R(1,1);

  return  point_map;
}

void PeopleDetectorHOGNode::publishResults(){
	// Publish the results
	sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_img_src).toImageMsg();
	m_image_pub_.publish(img_msg);

	if(m_peoplePositionsOnMap.size() > 0) {
		m_positioncov_pub_.publish(m_peoplePositionsOnMap[0]);	

		geometry_msgs::PointStamped point2d;
		point2d.header = m_peoplePositionsOnMap[0].header;
		point2d.point =  m_peoplePositionsOnMap[0].point.point;
		m_position_pub_.publish(point2d);
	}
	
}



// Main function
int main(int argc, char** argv)
{
	ros::init(argc, argv, "people_detector_hog_node");
  if (ros::names::remap("image")=="image") {
		ROS_WARN("Topic 'image' has not been remapped! Typical command-line usage:\n"
						 "\t$ rosrun people_detector people_detector_hog_node image:=<image topic> [transport]");
	}	

	PeopleDetectorHOGNode nodePeopleDetectorHOG;
 
  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  dynamic_reconfigure::Server<people_detector::vision_people_detector_hogConfig> dr_srv;
  dynamic_reconfigure::Server<people_detector::vision_people_detector_hogConfig>::CallbackType cb;
  cb = boost::bind(&PeopleDetectorHOGNode::configCallback, &nodePeopleDetectorHOG, _1, _2);
  dr_srv.setCallback(cb);

  ros::spin();

  return 0;
}
