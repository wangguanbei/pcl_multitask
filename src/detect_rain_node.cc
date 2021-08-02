#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <fstream>

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    float intensity;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (float, time, time)
)

typedef PointXYZIRT PointT;
typedef pcl::PointCloud<PointT> PointCloud;

int main(int argc, char **argv)
{
    std::ofstream outfile;
    PointT p_dry, p_rain;
    int ring_index, show_num;
    std::cin >> ring_index >> show_num;
    PointCloud::Ptr dry(new PointCloud);
    PointCloud::Ptr rain(new PointCloud);
    PointCloud::Ptr dry_ring(new PointCloud);
    PointCloud::Ptr rain_ring(new PointCloud);

    //加载点云文件
    pcl::io::loadPCDFile("/media/ubuwgb/Samsung_T5_WGB/apollo_rosbag/dry.pcd", *dry);
    pcl::io::loadPCDFile("/media/ubuwgb/Samsung_T5_WGB/apollo_rosbag/rain.pcd", *rain);

    outfile.open("/media/ubuwgb/Samsung_T5_WGB/apollo_rosbag/dry_ring.txt");
    for (int i = 0; i < dry->points.size(); i++)
    // for (int i = 0; i < show_num;i++)
    {
        p_dry = dry->points[i];
        if (p_dry.ring==ring_index){
            dry_ring->push_back(p_dry);
            outfile << p_dry.x
                    << ";" << p_dry.y
                    << ";" << p_dry.z
                    << ";" << p_dry.intensity
                    << ";" << p_dry.ring
                    << ";" << p_dry.time << "\n";
        }
        if (dry_ring->points.size()>=show_num)
            break;
        // std::cout << "p_dry.x:" << p_dry.x
        //      << ", p_dry.y:" << p_dry.y
        //      << ", p_dry.z:" << p_dry.z
        //      << ", p_dry.intensity:" << p_dry.intensity
        //      << ", p_dry.ring:" << p_dry.ring
        //      << ", p_dry.time:" << p_dry.time << "\n";
    }
    outfile.close();

    std::cout << "\n";

    outfile.open("/media/ubuwgb/Samsung_T5_WGB/apollo_rosbag/rain_ring.txt");
    for (int i = 0; i < rain->points.size();i++)
    // for (int i = 0; i < show_num;i++)
    {
        p_rain = rain->points[i];
        if (p_rain.ring==ring_index){
            rain_ring->push_back(p_rain);
            outfile << p_rain.x
                    << ";" << p_rain.y
                    << ";" << p_rain.z
                    << ";" << p_rain.intensity
                    << ";" << p_rain.ring
                    << ";" << p_rain.time << "\n";
        }
        if (rain_ring->points.size()>=show_num)
            break;
        // std::cout << "p_rain.x:" << p_rain.x
        //      << ", p_rain.y:" << p_rain.y
        //      << ", p_rain.z:" << p_rain.z
        //      << ", p_rain.intensity:" << p_rain.intensity
        //      << ", p_rain.ring:" << p_rain.ring
        //      << ", p_rain.time:" << p_rain.time << "\n";
    }
    outfile.close();

    //保存点云
    pcl::io::savePCDFileASCII("/media/ubuwgb/Samsung_T5_WGB/apollo_rosbag/dry_ring.pcd", *dry_ring);
    pcl::io::savePCDFileASCII("/media/ubuwgb/Samsung_T5_WGB/apollo_rosbag/rain_ring.pcd", *rain_ring);

    return 0;
}