#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> //error: ‘fromROSMsg’ is not a member of ‘pcl’

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    uint8_t intensity;
    uint8_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint8_t, intensity, intensity)
                                  (uint8_t, ring, ring)
                                  (double, timestamp, timestamp)
)

typedef PointXYZIRT PointT;
typedef pcl::PointCloud<PointT> PointCloud;
PointCloud::Ptr pclCloud(new PointCloud);
std::string outpcd_path;

void CloudHandler(const sensor_msgs::PointCloud2ConstPtr& cloudmsg)
{
    double time = cloudmsg->header.stamp.toSec();

    pclCloud->clear();
    pcl::fromROSMsg(*cloudmsg, *pclCloud);
    //保存点云
    std::string filename;
    filename = outpcd_path;
    filename += std::to_string(time);
    filename += ".pcd";
    std::cout << "filename: " << filename << std::endl;
    pcl::io::savePCDFileASCII(filename, *pclCloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bag2pcd");
    ros::NodeHandle nh;
    std::string input_topic;
    nh.getParam("input_topic", input_topic);
    nh.getParam("outpcd_path", outpcd_path);

    ros::Subscriber pc = nh.subscribe<sensor_msgs::PointCloud2>
                                    (input_topic, 5, CloudHandler);
    ros::spin();

    return 0;
}