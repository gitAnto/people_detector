# people_detector
Vision-based people detector for ROS  
Copyright (c) 2014-2016, Donato Di Paola.

## Package Summary
This package provides a generic structure for vision-based object detection algorithms in ROS.

## License

MIT License, See `LICENSE` file.


## Nodes

### vision\_people\_detector\_hog\_node
This node implements the generic vision-based class detector to detect people in an image stream by using the [OpenCV HoG object detector](http://docs.opencv.org/2.4/modules/gpu/doc/object_detection.html). The node returns the position of first person detected with respect to the camera frame.

#### Subscribed Topics
- `/camera_info` *sensor_msgs::CameraInfo*<br/>
  Camera intrinsics for images published on /image
- `/image` *sensor_msgs::Image*<br/>
  A stream of raw images from the camera 


#### Published Topics
- `/image_labeled` *sensor_msgs::Image*<br/>
  The stream of images with boxes over detected people.
- `/person_point` *geometry_msgs::PointStamped*<br/>
  Position of the first person detected in the environment
- `/person_pointcov` *people\_detector\_msgs:: PointWithCovarianceStamped*<br/>
  Position of the first person detected in the environment with Covariance
  
#### Params
- `image_scale` *double* (default "0.75")  
  The scale factor at which the input image is resized
- `hog_win_stride` *int* (default "8")  
  HoG Window stride
- `hog_scale0` *double* (default "1.05")  
  Coefficient of the HoG detection window



 
