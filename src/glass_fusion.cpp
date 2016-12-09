#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
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
#include <pcl/features/normal_3d.h>
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

using namespace sensor_msgs;
using namespace message_filters;
using namespace pcl;

ros::Publisher final_pub;

// PCL Visualizer to view the pointcloud
// pcl::visualization::PCLVisualizer viewer ("Door handle final");
  
// Initialise global Poincloud and Pointcloud pointer 8or else they dont work)
pcl::PointCloud<pcl::PointXYZ> output_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(&output_cloud);

void callback(const sensor_msgs::PointCloud2ConstPtr& rgbd, const sensor_msgs::PointCloud2ConstPtr& pcl)
{

  // Convert to XYZ type
  pcl::PointCloud<pcl::PointXYZ> cloud_rgbd;
  pcl::fromROSMsg(*rgbd, cloud_rgbd);
  pcl::PointCloud<pcl::PointXYZ> cloud_pcl;
  pcl::fromROSMsg(*pcl, cloud_pcl);

  // initialize output cloud and pointer

  output_cloud.clear();
  output_cloud.width = cloud_rgbd.size();
  output_cloud.height = 1;
  output_cloud.resize(cloud_rgbd.width);
  int counter = 0;

  for (size_t i = 0; i < cloud_rgbd.size(); i++){
      bool in_cloud_pcl = false;
      for (size_t j = 0; j < cloud_pcl.size(); j++){
	  float x_diff = std::abs(cloud_rgbd.points[i].x - cloud_pcl.points[j].x);
	  float y_diff = std::abs(cloud_rgbd.points[i].y - cloud_pcl.points[j].y);
	  float z_diff = std::abs(cloud_rgbd.points[i].z - cloud_pcl.points[j].z);
          //std::cout<< " x_diff = " << x_diff << " y_diff = " << y_diff << " z_diff = " << z_diff << std::endl;

          if ((x_diff<0.04) && (y_diff<0.04) && (z_diff<0.02)){
              in_cloud_pcl = true;
              break;
          }
      }

      if (in_cloud_pcl){
          //std::cout<< " Saving " <<std::endl;
          output_cloud.points[counter].x = cloud_rgbd.points[i].x;
          output_cloud.points[counter].y = cloud_rgbd.points[i].y;
          output_cloud.points[counter].z = cloud_rgbd.points[i].z;
          counter++;
      }
  }
  output_cloud.width = counter;
  output_cloud.resize(counter);
  
  // Publish the data
  ros::Time time_st = ros::Time::now ();
  output_cloud.header.stamp = time_st.toNSec()/1e3;
  output_cloud.header.frame_id = "camera_rgb_optical_frame";
  final_pub.publish (output_cloud);
 
/*   
  // Visualize pointcloud
  viewer.addCoordinateSystem();
  viewer.addPointCloud (point_cloud_ptr, "scene_cloud");
  viewer.spinOnce();
  viewer.removePointCloud("scene_cloud");
*/


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "glass_fusion");
  
  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/glass/pcl_seg/pcl", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> rgbd_sub(nh, "/glass/vis_seg/pcl", 1);
  final_pub = nh.advertise<sensor_msgs::PointCloud2> ("/glass/final/pcl", 1);


  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgbd_sub, pcl_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));


  ros::spin ();
  return 0;

}
