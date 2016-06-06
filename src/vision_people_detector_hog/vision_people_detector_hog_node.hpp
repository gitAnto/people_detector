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

#ifndef _VISION_PEOPLE_DETECTOR_HOG_NODE_HPP_
#define _VISION_PEOPLE_DETECTOR_HOG_NODE_HPP_

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <people_detector/PointWithCovarianceStamped.h>
#include "../common/vision_node.hpp"


class PeopleDetectorHOGNode : public VisionNode {			

	private:
		// ROS publisher declaretions
		image_transport::Publisher m_image_pub_;
		ros::Publisher m_positioncov_pub_;	
		ros::Publisher m_position_pub_;	

		// ROS tf declaretions
		tf::TransformListener m_tf_listener;

		// Publisher object declaretions
		cv::vector<cv::Rect> m_peopleBoxes;
		cv::vector<double> m_peopleWeights;
		cv::vector<people_detector::PointWithCovarianceStamped> m_peoplePositionsOnMap;

		// Dynamic reconfigure ros params
		int m_hog_win_stride;
		double m_hog_scale0; 
		
		// Node methods				
		cv::Mat getOutputImage();
		people_detector::PointWithCovarianceStamped getMapCoords(cv::Point2d point_img, ros::Time stamp);

	protected:
		void runVisionCore();
		void publishResults();

	public:
		PeopleDetectorHOGNode();
		~PeopleDetectorHOGNode();
		void configCallback(people_detector::vision_people_detector_hogConfig &config, uint32_t level);
};


#endif //_VISION_PEOPLE_DETECTOR_HOG_NODE_HPP_
