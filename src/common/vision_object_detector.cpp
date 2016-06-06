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

#include "vision_object_detector.hpp"

		
ObjectDetector::ObjectDetector(){
	m_obj_weight_treshold = 1.0;		
}

void ObjectDetector::detectObjects(const cv::Mat img){

	// Set the image
	m_img = img;

	// Run the specific object detector on the image
	runObjectDetector();

	// Run the specific object filter
	filterObjects();
	
}

void ObjectDetector::filterObjects() {
	// Find center and base midpoint for each box object
	cv::vector<cv::Rect> obj_boxes_filtered;
	size_t i;
	
	// SINGLE OBJECT FILTERING : select the maximum weight
	double maxweight = 0.0;
	size_t maxweight_idx= 0;

	for (i=0; i<m_obj_weights.size(); i++) 
		if (m_obj_weights[i] > maxweight) {
			maxweight = m_obj_weights[i];
			maxweight_idx = i;
    }  
	
	if (maxweight > m_obj_weight_treshold) {
		obj_boxes_filtered.clear();
		obj_boxes_filtered.push_back(m_obj_boxes[maxweight_idx]);
		m_obj_boxes = obj_boxes_filtered;
	} else {
		m_obj_boxes.clear();
	}
	
	for (int i=0; i<m_obj_boxes.size(); i++){

		// Reduce the box size (to fit the object better!)				
		cv::Rect r = m_obj_boxes[i];
		r.x += cvRound(r.width*0.1);
		r.width = cvRound(r.width*0.8);
		r.y += cvRound(r.height*0.1);
		r.height = cvRound(r.height*0.7);

		// Update the vector with the new boxes
    m_obj_boxes[i] = r;

		cv::Point2d obj_center(r.x + (r.width/2), r.y + (r.height/2)); 
		m_obj_centers.push_back(obj_center);
		
		cv::Point2d obj_base_midpoint(r.x + (r.width/2), r.y + (r.height));
		m_obj_base_midpoints.push_back(obj_base_midpoint);		  

	}
}

cv::vector<cv::Rect> ObjectDetector::getObjBoxes(){
	return  m_obj_boxes;
}

cv::vector<double> ObjectDetector::getObjWeights(){
	return  m_obj_weights;
}

cv::vector<cv::Point2d> ObjectDetector::getObjCentres(){
	return  m_obj_centers;
}

cv::vector<cv::Point2d> ObjectDetector::getObjBaseMidpoints(){
	return  m_obj_base_midpoints;
}

