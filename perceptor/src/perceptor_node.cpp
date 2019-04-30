#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/time_cache.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h> // this allows for interoperability
#include <pcl/common/centroid.h>

#include <string>

//#include <pcl_ros>
#include <pcl_ros/transforms.h>

//#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <pcl/filters/conditional_removal.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl;


#include <pcl/filters/voxel_grid.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>

#include <boost/foreach.hpp>

//tf::TransformListener listener;
boost::shared_ptr<tf::TransformListener> listener;


inline void PointCloudXYZRGBAtoXYZRGB(pcl::PointCloud<pcl::PointXYZRGBA> &in, pcl::PointCloud<pcl::PointXYZRGB> &out) {
    out.width = in.width;
    out.height = in.height;
    out.points.resize(in.points.size());
    for (size_t i = 0; i < in.points.size(); i++) {
        out.points[i].x = in.points[i].x;
        out.points[i].y = in.points[i].y;
        out.points[i].z = in.points[i].z;
        out.points[i].r = in.points[i].r;
        out.points[i].g = in.points[i].g;
        out.points[i].b = in.points[i].b;
    }
}


//http://docs.pointclouds.org/1.3.1/classpcl_1_1_color_filter_3_01sensor__msgs_1_1_point_cloud2_01_4.html

ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


void
cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    ROS_INFO_STREAM("helloworld");
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    BOOST_FOREACH (const pcl::PointXYZRGB& pt, cloud_msg->points)
//        printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGBA> temp_cloud;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, temp_cloud);
    PointCloudXYZRGBAtoXYZRGB(temp_cloud, *cloud);

    ROS_INFO_STREAM("cloud_size " << cloud->width << " " << cloud->height);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);


// build the condition 
    int rMax = 255;
    int rMin = 150;
    int gMax = 50;
    int gMin = 0;
    int bMax = 50;
    int bMin = 0;

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
    color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
            new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::LT, rMax)));
    color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
            new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, rMin)));
    color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
            new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::LT, gMax)));
    color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
            new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GT, gMin)));
    color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
            new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::LT, bMax)));
    color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
            new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GT, bMin)));

    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
    condrem.setCondition(color_cond);
    condrem.setInputCloud(cloud);
//    condrem.setKeepOrganized(true);

//    //// apply filter
    condrem.filter(*cloud_filtered);
    ROS_INFO_STREAM("filtered_cloud_size " << cloud_filtered->width << " " << cloud_filtered->height);

    ROS_INFO_STREAM("Hello" << std::endl);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

    tf::StampedTransform transform;
//    tf::Transform transform;


// this is supposed to work
    listener->waitForTransform("/world", "/multisense/head_root", ros::Time(0), ros::Duration(10.0));
    listener->lookupTransform("/world", "/multisense/head_root", ros::Time(0), transform);
//
    pcl_ros::transformPointCloud("/world", *cloud_filtered, *cloud_filtered_transformed, *listener);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_filtered_transformed, centroid);

    ROS_INFO_STREAM("CENTROID IS: " << std::endl << centroid << std::endl);


    // Publish the data
//    pub.publish(*cloud_filtered);
//    pub.publish(output);
}

int
main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "perceptor_node");
    ros::NodeHandle nh;
    listener.reset(new (tf::TransformListener));
    // Create a ROS subscriber for the input point cloud
    //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("multisense/camera/points2", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("hello/output", 1);

    Eigen::MatrixXd m(2, 2);
    m(0, 0) = 3;
    m(1, 0) = 2.5;
    m(0, 1) = -1;
    m(1, 1) = m(1, 0) + m(0, 1);
    //std::cout << m << std::endl;
    ROS_INFO_STREAM("matrix is: " << m);

    // Spin
    ros::spin();
}




// **************




//#include <ros/ros.h>
//// PCL specific includes
//#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

//#include <iostream>
//#include <pcl/filters/conditional_removal.h>
//#include <pcl/console/parse.h>

//using namespace std; 
//using namespace pcl; 



//#include <pcl/filters/voxel_grid.h>

////Eigen
//#include <Eigen/Core>
//#include <Eigen/Eigen>
//#include <Eigen/Dense>
//#include <Eigen/Geometry>
//#include <Eigen/Eigenvalues>
//#include <Eigen/Cholesky>
//#include <Eigen/StdVector>


//ros::Publisher pub;

//void 
//cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
//{
//ROS_INFO_STREAM("helloworld");
////pcl::PointXYZRGB

//// Container for original & filtered data
//pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
//pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//pcl::PCLPointCloud2 cloud_filtered;

////pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
////pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
////pcl::PCLPointCloud2 cloud_filtered;

//// Convert to PCL data type
//pcl_conversions::toPCL(*cloud_msg, *cloud);


//// build the condition 
//int rMax = 255;
//int rMin = 150;
//int gMax = 50;
//int gMin = 0;
//int bMax = 50;
//int bMin = 0;

////pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
////color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMax)));
////color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rMin)));
////color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gMax)));
////color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMin)));
////color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bMax)));
////color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMin)));


////pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
////color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMax)));
////color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rMin)));
////color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gMax)));
////color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMin)));
////color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bMax)));
////color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMin)));

//// build the filter
////pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (color_cond);
////condrem.setInputCloud (cloud);
////condrem.setKeepOrganized(true);

//// build the filter
////pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
////condrem.setCondition (color_cond);
////condrem.setInputCloud (cloud);
////condrem.setKeepOrganized(true);

////// apply filter
////condrem.filter (*cloud_filtered);

//// Perform the actual filtering
//pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//sor.setInputCloud (cloudPtr);
//float leaf = 0.2;
//sor.setLeafSize (leaf,leaf, leaf);
//sor.filter (cloud_filtered);

//// Convert to ROS data type
//sensor_msgs::PointCloud2 output;
//pcl_conversions::moveFromPCL(cloud_filtered, output);

//// Publish the data
//pub.publish (output);
//}

//int
//main (int argc, char** argv)
//{
//// Initialize ROS
//ros::init (argc, argv, "perceptor_node");
//ros::NodeHandle nh;

//// Create a ROS subscriber for the input point cloud
////ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);

//ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("multisense/camera/points2", 1, cloud_cb);

//// Create a ROS publisher for the output point cloud
//pub = nh.advertise<sensor_msgs::PointCloud2> ("hello/output", 1);

//Eigen::MatrixXd m(2,2);
//m(0,0) = 3;
//m(1,0) = 2.5;
//m(0,1) = -1;
//m(1,1) = m(1,0) + m(0,1);
////std::cout << m << std::endl;
//ROS_INFO_STREAM("matrix is: " << m);

//// Spin
//ros::spin ();
//}
