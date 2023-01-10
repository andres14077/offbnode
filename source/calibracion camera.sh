rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 8x6 --square 2.0 right:=/offbnode/right/image_raw left:=/offbnode/left/image_raw


rosrun image_view stereo_view stereo:=offbnode image:=image_rect

ROS_NAMESPACE=offbnode rosrun stereo_image_proc stereo_image_proc