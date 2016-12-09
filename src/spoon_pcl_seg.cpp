#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cmath> 

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace pcl;
using namespace cv;

ros::Publisher img_pub;
ros::Publisher depth_pub;
ros::Publisher pcl_full_pub;
ros::Publisher pcl_pub;

// Global calibration values from Kinect (depth)
float fx_rgb = 542.874146;
float fy_rgb = 546.720581;
float cx_rgb = 320.693510;
float cy_rgb = 264.239638;
float fx_d = 605.119568;
float fy_d = 605.215271;
float cx_d = 298.887004;
float cy_d = 264.589174;
/*float fx_rgb = 524.8658126516821;
float fy_rgb = 526.0833409797511;
float cx_rgb = 312.2262287922412;
float cy_rgb = 255.4394087221328;
float fx_d = 595.1658098859201;
float fy_d = 596.9074027626567;
float cx_d = 310.6772546302307;
float cy_d = 247.6954910343287;*/


void callback(const sensor_msgs::ImageConstPtr& mask_sub, const sensor_msgs::ImageConstPtr& img_sub, const sensor_msgs::ImageConstPtr& depth_sub)
{

  cv_bridge::CvImagePtr mask_ptr;
  cv_bridge::CvImagePtr img_ptr;
  cv_bridge::CvImagePtr depth_ptr;
  try
  {
    mask_ptr = cv_bridge::toCvCopy(mask_sub, sensor_msgs::image_encodings::MONO8);
    img_ptr = cv_bridge::toCvCopy(img_sub, sensor_msgs::image_encodings::BGR8);
    depth_ptr = cv_bridge::toCvCopy(depth_sub, "32FC1");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Update GUI Window
  //cv::imshow("Mask viewer", mask_ptr->image);
  //cv::imshow("Image viewer", img_cropped);
  //cv::imshow("Depth viewer", depth_cropped);
  //cv::waitKey(1);

// Create conversions
  Mat mask_src;
  mask_src=mask_ptr->image;
  Mat img_src, img_cropped;
  img_src=img_ptr->image;
  Mat depth_src, depth_cropped;
  depth_src=depth_ptr->image;

  // Generate cropped image
  img_src.copyTo(img_cropped, mask_src);

  // Generate cropped depth
  depth_src.copyTo(depth_cropped, mask_src);

  // Generate cropped pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr fullPointcloud (new pcl::PointCloud <pcl::PointXYZ>); 
  float rgbFocalInvertedX = 1/fx_rgb;	// 1/fx
  float rgbFocalInvertedY = 1/fy_rgb;	// 1/fy

  pcl::PointXYZ newPoint;
  for (int i=0;i<depth_cropped.rows;i++)
  {
     for (int j=0;j<depth_cropped.cols;j++)
     {
       float depthValue = depth_cropped.at<float>(i,j);
       if (depthValue != 0)                // if depthValue is not NaN
       {
         // Find 3D position respect to rgb frame:
	 newPoint.z = depthValue/1000;
	 newPoint.x = (j - cx_rgb) * newPoint.z * rgbFocalInvertedX;
	 newPoint.y = (i - cy_rgb) * newPoint.z * rgbFocalInvertedY;
	 fullPointcloud->push_back(newPoint);
	 }
      }
   }

 
  //Convert images to ROS type
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  cv_bridge::CvImage img_bridge;
  cv_bridge::CvImage depth_bridge;
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img_cropped);
  depth_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, depth_cropped);

  // Publish images
  img_pub.publish(img_bridge.toImageMsg());
  depth_pub.publish(depth_bridge.toImageMsg());

  // Publish pointcloud
  fullPointcloud->header.frame_id = "camera_rgb_optical_frame";
  ros::Time time_st = ros::Time::now ();
  fullPointcloud->header.stamp = time_st.toNSec()/1e3;
  pcl_full_pub.publish (fullPointcloud);

  // ************************** Planar segmentation *************************  

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_spoon (new pcl::PointCloud<pcl::PointXYZ> ());
  
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  //seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.0045);
  seg.setInputCloud (fullPointcloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
  std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (fullPointcloud);
  extract.setIndices (inliers);
  extract.setNegative (false);

  
  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_spoon);
  

  // Publish the inlier data (table)
  time_st = ros::Time::now ();
  cloud_spoon->header.frame_id = "camera_rgb_optical_frame";
  cloud_spoon->header.stamp = time_st.toNSec()/1e3;
  pcl_pub.publish (cloud_spoon);
/*
  // Display Pointcloud
  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (cloud_spoon);
   while (!viewer.wasStopped ())
   {
   }
 */ 
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "spoon_pcl_seg");
/*
  cv::namedWindow("Mask viewer");
  cv::namedWindow("Image viewer");
  cv::namedWindow("Depth viewer");
*/
  
  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image> mask_sub(nh, "/cropped/spoon_mask", 1);
  message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/camera/rgb/image_rect_color", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/hw_registered/image_rect_raw", 1);
  pcl_full_pub = nh.advertise<sensor_msgs::PointCloud2> ("/spoon/full_roi_pcl", 1);
  pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/spoon/pcl_seg/pcl", 1);
  img_pub = nh.advertise<sensor_msgs::Image> ("/spoon/spoon_rect_color", 1);
  depth_pub = nh.advertise<sensor_msgs::Image> ("/spoon/spoon_rect_depth", 1);

  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(7), mask_sub, img_sub, depth_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));


  ros::spin ();
  return 0;

}
