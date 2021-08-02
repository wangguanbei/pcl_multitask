#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> //error: ‘fromROSMsg’ is not a member of ‘pcl’
#include <math.h>                            //PI

#include "buildMapWithGPS/gnss_bestpose.h"
#include "buildMapWithGPS/gnss_heading.h"

using std::cout;
using std::endl;

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    uint8_t intensity;
    uint8_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(uint8_t, ring, ring)(double, timestamp, timestamp))

// typedef PointXYZIRT PointT;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//some variable
PointCloud::Ptr curCloud(new PointCloud);
std::string outpcd_path;
double init_utmx, init_utmy, init_heading;
double utmx, utmy;
PointCloud::Ptr mapBuildByGPS(new PointCloud);
ros::Publisher pubmapBuildByGPS;
double lidarIngps_lon, lidarIngps_lat, lidarIngps_yaw_clockwise;

//bestpose variable
bool bestpose_new = false;
double timestamp_bestpose;
double sol_status;
double sol_type;
double latitude;
double longitude;
double height;
double num_sats_in_solution;
int gps_count = 0; //记录 GPS 数据到来的次数

//heading variable
bool heading_new = false;
double timestamp_heading;
double heading;
double heading_std_dev;
int heading_count = 0; //记录 heading 数据到来的次数

void LonLat2UTM(double longitude, double latitude, double &UTME, double &UTMN)
{
    double lat = latitude;
    double lon = longitude;

    double kD2R = M_PI / 180.0;
    double ZoneNumber = floor((lon - 1.5) / 3.0) + 1;
    double L0 = ZoneNumber * 3.0;

    double a = 6378137.0;
    double F = 298.257223563;
    double f = 1 / F;
    double b = a * (1 - f);
    double ee = (a * a - b * b) / (a * a);
    double e2 = (a * a - b * b) / (b * b);
    double n = (a - b) / (a + b);
    double n2 = (n * n);
    double n3 = (n2 * n);
    double n4 = (n2 * n2);
    double n5 = (n4 * n);
    double al = (a + b) * (1 + n2 / 4 + n4 / 64) / 2.0;
    double bt = -3 * n / 2 + 9 * n3 / 16 - 3 * n5 / 32.0;
    double gm = 15 * n2 / 16 - 15 * n4 / 32;
    double dt = -35 * n3 / 48 + 105 * n5 / 256;
    double ep = 315 * n4 / 512;
    double B = lat * kD2R;
    double L = lon * kD2R;
    L0 = L0 * kD2R;
    double l = L - L0;
    double cl = (cos(B) * l);
    double cl2 = (cl * cl);
    double cl3 = (cl2 * cl);
    double cl4 = (cl2 * cl2);
    double cl5 = (cl4 * cl);
    double cl6 = (cl5 * cl);
    double cl7 = (cl6 * cl);
    double cl8 = (cl4 * cl4);
    double lB = al * (B + bt * sin(2 * B) + gm * sin(4 * B) + dt * sin(6 * B) + ep * sin(8 * B));
    double t = tan(B);
    double t2 = (t * t);
    double t4 = (t2 * t2);
    double t6 = (t4 * t2);
    double Nn = a / sqrt(1 - ee * sin(B) * sin(B));
    double yt = e2 * cos(B) * cos(B);
    double N = lB;
    N = N + t * Nn * cl2 / 2;
    N = N + t * Nn * cl4 * (5 - t2 + 9 * yt + 4 * yt * yt) / 24;
    N = N + t * Nn * cl6 * (61 - 58 * t2 + t4 + 270 * yt - 330 * t2 * yt) / 720;
    N = N + t * Nn * cl8 * (1385 - 3111 * t2 + 543 * t4 - t6) / 40320;
    double E = Nn * cl;
    E = E + Nn * cl3 * (1 - t2 + yt) / 6;
    E = E + Nn * cl5 * (5 - 18 * t2 + t4 + 14 * yt - 58 * t2 * yt) / 120;
    E = E + Nn * cl7 * (61 - 479 * t2 + 179 * t4 - t6) / 5040;
    E = E + 500000;
    N = 0.9996 * N;
    E = 0.9996 * (E - 500000.0) + 500000.0;

    UTME = E;
    UTMN = N;
}

void CloudHandler(const sensor_msgs::PointCloud2ConstPtr &cloudmsg)
{
    if (bestpose_new && heading_new && fabs(timestamp_bestpose - timestamp_heading)<0.1)
    {
        bestpose_new = false;
        heading_new = false;
    }
    else
        return;
    cout << std::setprecision(15) << timestamp_bestpose << "\t;" << utmx << "\t;"
              << utmy << "\t;" << height << ";" << gps_count << endl;
    cout << std::setprecision(15) << timestamp_heading << "\t;" << heading << "\t;"
              << init_heading << "\t;" << heading_count << "\n\n";
    curCloud->clear();
    pcl::fromROSMsg(*cloudmsg, *curCloud);
    double xInCar, yInCar;

    for (int i = 0; i < curCloud->points.size();i++)
    {
        PointT transformed_point;
        xInCar = curCloud->points[i].y * cos(lidarIngps_yaw_clockwise / 180 * M_PI) -
                 curCloud->points[i].x * sin(lidarIngps_yaw_clockwise / 180 * M_PI) +
                 lidarIngps_lat;
        yInCar = -curCloud->points[i].x * cos(lidarIngps_yaw_clockwise / 180 * M_PI) -
                 curCloud->points[i].y * sin(lidarIngps_yaw_clockwise / 180 * M_PI) +
                 lidarIngps_lon;
        transformed_point.x = utmx * cos(init_heading / 180 * M_PI) -
                              utmy * sin(init_heading / 180 * M_PI) +
                              xInCar * cos(heading / 180 * M_PI) +
                              yInCar * sin(heading / 180 * M_PI);
        transformed_point.y = utmx * sin(init_heading / 180 * M_PI) +
                              utmy * cos(init_heading / 180 * M_PI) -
                              xInCar * sin(heading / 180 * M_PI) +
                              yInCar * cos(heading / 180 * M_PI);
        transformed_point.z = curCloud->points[i].z;
        mapBuildByGPS->points.push_back(transformed_point);
    }

    sensor_msgs::PointCloud2 cloudMsgTemp;
    mapBuildByGPS->width = 1;
    mapBuildByGPS->height = mapBuildByGPS->points.size();
    pcl::toROSMsg(*mapBuildByGPS, cloudMsgTemp);
    cloudMsgTemp.header.stamp = cloudmsg->header.stamp;
    cloudMsgTemp.header.frame_id = "/velodyne";
    pubmapBuildByGPS.publish(cloudMsgTemp);
    // //保存点云
    double time = cloudmsg->header.stamp.toSec();
    if(time>1622104010)
    {
        std::string filename;
        filename = outpcd_path;
        filename += std::to_string(time);
        filename += ".pcd";
        cout << "filename: " << filename << endl;
        pcl::io::savePCDFileASCII(filename, *mapBuildByGPS);
    }
}

void bestposeCallback(const geometry_msgs::gnss_bestpose::ConstPtr &bestpose_msg)
{
    // ROS_INFO("I heard: [%s]", msg->data.c_str());

    timestamp_bestpose = bestpose_msg->header.stamp.sec + bestpose_msg->header.stamp.nsec * (1e-9);
    sol_status = bestpose_msg->sol_status;
    sol_type = bestpose_msg->sol_type;
    latitude = bestpose_msg->latitude;
    longitude = bestpose_msg->longitude;
    height = bestpose_msg->height_msl + bestpose_msg->undulation;
    num_sats_in_solution = bestpose_msg->num_sats_in_solution;
    // cout << std::setprecision(15) << timestamp_bestpose << ";" << sol_status << ";"
    //           << sol_type << ";" << latitude << ";" << longitude << ";" << height << ";"
    //           << num_sats_in_solution << ";" << gps_count << endl;
    LonLat2UTM(longitude, latitude, utmx, utmy);
    if (gps_count == 0)
    {
        init_utmx = utmx;
        init_utmy = utmy;
    }
    utmx = utmx - init_utmx;
    utmy = utmy - init_utmy;
    gps_count = gps_count + 1; //记录gps数据到来次数
    // cout << std::setprecision(15) << timestamp_bestpose << ";" << utmx << ";"
    //           << utmy << ";" << height << ";" << gps_count << endl;
    if (!bestpose_new)
        bestpose_new = true;
}

void headingCallback(const geometry_msgs::gnss_heading::ConstPtr &heading_msg)
{
    // ROS_INFO("I heard: [%s]", msg->data.c_str());

    timestamp_heading = heading_msg->header.stamp.sec + heading_msg->header.stamp.nsec * (1e-9);
    heading = heading_msg->heading;
    heading_std_dev = heading_msg->heading_std_dev;
    // cout << std::setprecision(15) << timestamp_heading
    //           << ";" << heading << ";" << heading_std_dev << endl;
    if (heading_count == 0)
    {
        init_heading = heading;
    }
    heading = heading - init_heading;
    if (heading < 0)
    {
        heading = heading + 360;
    }
    heading_count = heading_count + 1; //记录gps数据到来次数
    // cout << std::setprecision(15) << timestamp_heading << ";" << heading << ";"
    //           << heading_count << endl;
    if (!heading_new)
        heading_new = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "buildMapWithGPS");
    ros::NodeHandle nh;
    std::string pc_input_topic, gps_topic, heading_topic;
    nh.getParam("pc_input_topic", pc_input_topic);
    nh.getParam("gps_topic", gps_topic);
    nh.getParam("heading_topic", heading_topic);
    nh.getParam("outpcd_path", outpcd_path);
    nh.getParam("lidarIngps_lon", lidarIngps_lon);
    nh.getParam("lidarIngps_lat", lidarIngps_lat);
    nh.getParam("lidarIngps_yaw_clockwise", lidarIngps_yaw_clockwise);
    mapBuildByGPS->clear();

    ros::Subscriber pc_sub = nh.subscribe<sensor_msgs::PointCloud2>(pc_input_topic, 5, CloudHandler);

    ros::Subscriber bestpose_sub = nh.subscribe("/gnss_bestpose", 1000, bestposeCallback);

    ros::Subscriber heading_sub = nh.subscribe("/gnss_heading", 1000, headingCallback);
    
    pubmapBuildByGPS = nh.advertise<sensor_msgs::PointCloud2>("/mapBuildByGPS", 2);
    // spin
    ros::spin();

    return 0;
}