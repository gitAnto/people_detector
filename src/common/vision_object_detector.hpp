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

#ifndef _VISION_OBJECT_DETECTOR_HPP_
#define _VISION_OBJECT_DETECTOR_HPP_

#include <opencv2/opencv.hpp>

class ObjectDetector {

	protected:
		cv::Mat m_img;

		cv::vector<cv::Rect> m_obj_boxes;
		cv::vector<double> m_obj_weights;
		double m_obj_weight_treshold;	

		cv::vector<cv::Point2d> m_obj_centers;
		cv::vector<cv::Point2d> m_obj_base_midpoints;
					

		virtual void runObjectDetector() = 0;		

		void filterObjects();		

	public:
		ObjectDetector();
		void detectObjects(const cv::Mat img);
		cv::vector<cv::Rect> getObjBoxes();
		cv::vector<double> getObjWeights();
		cv::vector<cv::Point2d> getObjCentres();
		cv::vector<cv::Point2d> getObjBaseMidpoints();
};

#endif //_VISION_OBJECT_DETECTOR_HPP_
