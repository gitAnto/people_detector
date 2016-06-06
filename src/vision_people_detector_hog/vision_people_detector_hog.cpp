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

#include "vision_people_detector_hog.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>


PeopleDetectorHOG::PeopleDetectorHOG(int win_stride, double scale0){
	// Set the algorithm params			
	m_win_stride = win_stride;
	m_scale0 = scale0;

	// Initialize the HOG People Detector
	m_hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
}

PeopleDetectorHOG::~PeopleDetectorHOG(){
}

void PeopleDetectorHOG::runObjectDetector(){
	// Run the HOG People Detector
	m_hog.detectMultiScale(m_img, m_obj_boxes, m_obj_weights, 0, cv::Size(m_win_stride,m_win_stride), cv::Size(32,32), m_scale0, 2, false);
}	

cv::vector<cv::Point2d> PeopleDetectorHOG::getPeoplePositions(){
	return m_obj_base_midpoints;
}


