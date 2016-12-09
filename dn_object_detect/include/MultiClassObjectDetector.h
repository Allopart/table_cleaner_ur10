//
//  MultiClassObjectDetector.h
//  pr2_perception
//
//  Created by Xun Wang on 12/05/16.
//  Copyright (c) 2016 Xun Wang. All rights reserved.
//

#ifndef __pr2_perception__MultiClassObjectDetector__
#define __pr2_perception__MultiClassObjectDetector__

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/barrier.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <darknet/yolo.h>

#include "dn_object_detect/ObjectInfo.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;
using namespace ros;

namespace uts_perp {

typedef std::vector<dn_object_detect::ObjectInfo> DetectedList;

class MultiClassObjectDetector
{
public:
  MultiClassObjectDetector();
  virtual ~MultiClassObjectDetector();
  
  void init();
  void fini();

  void continueProcessing();

private:
  NodeHandle priImgNode_;
  image_transport::ImageTransport imgTrans_;
  image_transport::Publisher imgPub_;
  image_transport::Publisher croppedimgPub_;
  image_transport::Publisher cropped_dish_img_pub;
  image_transport::Publisher cropped_fork_img_pub;
  image_transport::Publisher cropped_glass_img_pub;
  image_transport::Publisher cropped_knife_img_pub;
  image_transport::Publisher cropped_spoon_img_pub;
  image_transport::Subscriber imgSub_;
  Publisher mask_coeff_pub;
  Publisher mask_coeff_dish_pub;
  Publisher mask_coeff_fork_pub;
  Publisher mask_coeff_glass_pub;
  Publisher mask_coeff_knife_pub;
  Publisher mask_coeff_spoon_pub;
  Publisher dtcPub_;

  bool doDetection_;
  bool initialised_;
  int debugRequests_;
  int srvRequests_;

  float threshold_;

  boost::mutex mutex_;
  
  boost::thread * object_detect_thread_;
  
  sensor_msgs::ImageConstPtr imgMsgPtr_;

  std::string cameraDevice_;

  CallbackQueue imgQueue_;
  
  AsyncSpinner * procThread_;
  
  cv_bridge::CvImagePtr cv_ptr_;

  cv::Mat imgcopy;  
  cv::Mat imgcrop; 
    
  network * darkNet_;
  detection_layer detectLayer_;
  int maxNofBoxes_;

  void processingRawImages( const sensor_msgs::ImageConstPtr& msg );

  void startDetection();
  void stopDetection();

  void startDebugView();
  void stopDebugView();

  void doObjectDetection();

  void consolidateDetectedObjects( const image * im, box * boxes,
      float **probs, DetectedList & objList );
  void publishDetectedObjects( const DetectedList & objs );
  void drawDebug( const DetectedList & objs );
};
  
} // namespace uts_perp

#endif /* defined(__pr2_perception__MultiClassObjectDetector__) */
