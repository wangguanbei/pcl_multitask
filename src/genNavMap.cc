#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> //error: ‘fromROSMsg’ is not a member of ‘pcl’
#include <nav_msgs/OccupancyGrid.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h> //error: ‘pcl::visualization’ has not been declared
#include "tictoc.h"

using std::cout;
using std::endl;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

ros::Publisher pubGridMap;
double resolution, lidar_z;
int width, height;

void CloudHandler(const sensor_msgs::PointCloud2ConstPtr &cloudmsg)
{
    TicToc t_genGridmap(true);
    nav_msgs::OccupancyGrid gridmap;
    gridmap.header.frame_id = "velodyne";
    gridmap.header.stamp = cloudmsg->header.stamp;
    gridmap.info.resolution = resolution; // float32
    gridmap.info.width = width;       // uint32
    gridmap.info.height = height;      // uint32
    gridmap.data.resize(gridmap.info.width * gridmap.info.height, -1);

    double laserCloudRawTime = cloudmsg->header.stamp.toSec();
    PointCloud::Ptr oripclCloud(new PointCloud);
    pcl::fromROSMsg(*cloudmsg, *oripclCloud);

    PointCloud::Ptr pclCloudori2(new PointCloud);
	pcl::PassThrough<PointT> pass; // 声明直通滤波
	pass.setInputCloud(oripclCloud); // 传入点云数据
	pass.setFilterFieldName("y"); // 设置操作的坐标轴
    pass.setFilterLimits(width * resolution * -0.5, width * resolution * 0.5); // 设置坐标范围
    // pass.setFilterLimitsNegative(true); // 保留数据函数
    pass.filter(*pclCloudori2);  // 进行滤波输出

    PointCloud::Ptr pclCloudori3(new PointCloud);
	pass.setInputCloud(pclCloudori2); // 传入点云数据
	pass.setFilterFieldName("x"); // 设置操作的坐标轴
    pass.setFilterLimits(height * resolution * -0.5, height * resolution * 0.5); // 设置坐标范围
    // pass.setFilterLimitsNegative(true); // 保留数据函数
    pass.filter(*pclCloudori3);  // 进行滤波输出

    for (int i = 0; i < pclCloudori3->points.size(); i++)
    {
        // x-left, y-behind
        double x = -pclCloudori3->points[i].y;
        double y = -pclCloudori3->points[i].x;
        double z = pclCloudori3->points[i].z;

        if (z > -lidar_z && sqrt(x * x + y * y) > 0.5)
        {
            int gridx = int(x / resolution) + 0.5 * width;
            int gridy = int(y / resolution) + 0.5 * height;
            gridmap.data[gridx * width + gridy] = z + lidar_z;
        }
    }
    pubGridMap.publish(gridmap);

    t_genGridmap.toc("genGridMap");
}

int main(int argc, char **argv)
{
    cout.setf(std::ios::fixed, std::ios::floatfield);
    ros::init(argc, argv, "genNavMap");
    ros::NodeHandle nh;
    std::string input_topic;
    nh.getParam("input_topic", input_topic);
    nh.getParam("resolution", resolution);
    nh.getParam("width", width);
    nh.getParam("height", height);
    nh.getParam("lidar_z", lidar_z);

    pubGridMap = nh.advertise<nav_msgs::OccupancyGrid>("/gridmap_lidar", 2);
    
    ros::Subscriber pc = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 1000, CloudHandler);

    while(ros::ok())
    {
        ros::spin();
    }

    return 0;
}