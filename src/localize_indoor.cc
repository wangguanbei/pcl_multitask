#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> //error: ‘fromROSMsg’ is not a member of ‘pcl’
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include "localize_indoor/Scancontext.cpp"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h> //error: ‘pcl::visualization’ has not been declared

using std::cout;
using std::endl;
typedef pcl::PointXYZI PointType;

struct gtdata
{
    double time;
    double x;
    double y;
};

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

PointCloud::Ptr pclCloud(new PointCloud);
PointCloud::Ptr pcl2DCloud(new PointCloud);
PointCloud::Ptr gtmapCloud(new PointCloud);
Eigen::Matrix4f lastregisMat;
bool firstmsg;
SCManager scManager;
ros::Publisher pubRelocalTarget, pubRelocalSource, pubRelocalregis;
float x, y, z, roll, pitch, yaw;
ofstream txtlidar;
std::vector<float> map_list;

extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 2000;
// extern const float ang_res_x = 0.18;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0+1.0;
int frameIndex;

// registration params
int max_iter;
double translation_epsilon;
double step_size, reso;
std::string regis_type;
bool converged;
double fitnessScore;
double base_height;

//点云可视化
void visualize_pcd(PointCloud::Ptr pcd_src,
                   PointCloud::Ptr pcd_tgt,
                   PointCloud::Ptr pcd_final)
{
    //int vp_1, vp_2;
    // Create a PCLVisualizer object
    pcl::visualization::PCLVisualizer viewer("registration Viewer");
    //viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
    // viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h(pcd_src, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h(pcd_tgt, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> final_h(pcd_final, 0, 255, 0);
    viewer.addPointCloud(pcd_src, src_h, "source cloud");
    viewer.addPointCloud(pcd_tgt, tgt_h, "target cloud");
    viewer.addPointCloud(pcd_final, final_h, "source->transformed cloud");
    ifstream readcorr;
    readcorr.open("/home/ubuwgb/icp_internal/pcd/corres.txt");
    int corrnum = 0;
    cin >> corrnum;
    float x, y, z;
    for (int i = 0; i < corrnum;i++)
    {
        readcorr >> x >> y >> z;
        PointT soup(0.0);
        soup.x=x;
        soup.y=y;
        soup.z=z;
        readcorr >> x >> y >> z;
        PointT tarp(0.0);
        tarp.x=x;
        tarp.y=y;
        tarp.z=z;
        if (i%2!=0)
            continue;
        viewer.addLine<PointT>(soup, tarp, 0, 255, 0, std::to_string(i));
    }

    readcorr.close();

    //viewer.addCoordinateSystem(1.0);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

void savetxtCloudHandler(const sensor_msgs::PointCloud2ConstPtr &cloudmsg)
{
    std::vector<PointCloud> laserCloudScans(N_SCAN);
    PointCloud::Ptr laserCloud(new PointCloud);
    frameIndex++;
    txtlidar.open("/home/ubuwgb/matlab_data/2dlidar_" + std::to_string(frameIndex) + ".txt");
    double laserCloudRawTime = cloudmsg->header.stamp.toSec();
    // cout << endl;
    // cout << "laserCloudRawTime: " << laserCloudRawTime << endl;

    pclCloud->clear();
    pcl2DCloud->clear();
    pcl::fromROSMsg(*cloudmsg, *pclCloud);
    PointType p_velo;
    float verticalAngle, horizonAngle;
    size_t rowIdn, columnIdn;
    for (int i = 0; i < pclCloud->points.size(); i++)
    {
        p_velo = pclCloud->points[i];

        verticalAngle = atan2(p_velo.z, sqrt(p_velo.x * p_velo.x + p_velo.y * p_velo.y)) * 180 / M_PI;
        rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
        if (rowIdn < 0 || rowIdn >= N_SCAN)
        {
            cout << "out of 16: " << rowIdn << endl;
            continue;
        }
        pclCloud->points[i].intensity = rowIdn;

        horizonAngle = atan2(p_velo.x, p_velo.y) * 180 / M_PI;
        columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;
        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
        {
            cout << "out of 2000: " << columnIdn << endl;
            continue;
        }
        laserCloudScans[rowIdn].push_back(p_velo);

        // 2d
        if (verticalAngle < 1.5 && verticalAngle > 0.5)
        {
            txtlidar << p_velo.x << ","
                     << p_velo.y << ","
                     << p_velo.z << ","
                     << p_velo.intensity << "\n";
        }
    }
    // 2d
    txtlidar.close();
    return;
    for (int i = 0; i < N_SCAN; i++)
    {
        *laserCloud += laserCloudScans[i];
    }
    for (int i = 0; i < laserCloud->points.size(); i++)
    {
        p_velo = laserCloud->points[i];
        txtlidar << p_velo.x << ","
                 << p_velo.y << ","
                 << p_velo.z << ","
                 << p_velo.intensity << "\n";
    }
    // {
    //     sensor_msgs::PointCloud2 cloudMsgTemp;
    //     pcl::toROSMsg(*pclCloud, cloudMsgTemp);
    //     cloudMsgTemp.header.stamp = ros::Time().fromSec(laserCloudRawTime);
    //     cloudMsgTemp.header.frame_id = "/velodyne";
    //     pubRelocalregis.publish(cloudMsgTemp);
    // }
    txtlidar.close();
    return;
}

void CloudHandler(const sensor_msgs::PointCloud2ConstPtr &cloudmsg)
{
    TicToc t_localize(true);
    double laserCloudRawTime = cloudmsg->header.stamp.toSec();
    cout << endl;
    cout << "laserCloudRawTime: " << laserCloudRawTime << endl;

    pclCloud->clear();
    pcl2DCloud->clear();
    pcl::fromROSMsg(*cloudmsg, *pclCloud);
    PointType p_velo;
    for (int i = 0; i < pclCloud->points.size(); i++)
    {
        if (sqrt(pclCloud->points[i].x * pclCloud->points[i].x + pclCloud->points[i].y * pclCloud->points[i].y) < 1)
        {
            continue;
        }
        if (sqrt(pclCloud->points[i].x * pclCloud->points[i].x + pclCloud->points[i].y * pclCloud->points[i].y) > 15)
        {
            continue;
        }
        if (pclCloud->points[i].z < base_height || pclCloud->points[i].z > 2.3)
        {
            continue;
        }
        p_velo.x = pclCloud->points[i].x;
        p_velo.y = pclCloud->points[i].y;
        p_velo.z = 0;
        pcl2DCloud->push_back(p_velo);
    }

    // Align clouds
    Eigen::Matrix4f regisInitialMat;
    Eigen::Affine3f relocal_tune;
    if (firstmsg)
    {
        Eigen::Affine3f regisInitialMatFoo = pcl::getTransformation(x, y, z, roll, pitch, yaw);
        regisInitialMat = regisInitialMatFoo.matrix();
        firstmsg = false;
    }
    else
    {
        regisInitialMat = lastregisMat;
    }
    pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
    
    if (regis_type == "icp")
    {
        pcl::IterativeClosestPoint<PointT, PointT> regis_relocal;
        regis_relocal.setMaxCorrespondenceDistance(0.5);
        regis_relocal.setMaximumIterations(max_iter);
        regis_relocal.setTransformationEpsilon(translation_epsilon*translation_epsilon);
        regis_relocal.setEuclideanFitnessEpsilon(1e-2);
        regis_relocal.setRANSACIterations(0);
        // regis_relocal.setInputSource(pclCloud);
        regis_relocal.setInputSource(pcl2DCloud);
        regis_relocal.setInputTarget(gtmapCloud);
        // regis_relocal.align(*unused_result);
        regis_relocal.align(*unused_result, regisInitialMat); // PCL regis non-eye initial is bad ... don't use (LeGO LOAM author also said pcl transform is weird.)
        lastregisMat = regis_relocal.getFinalTransformation().matrix();
        cout << "[indoor_localize] regis fit score: " << regis_relocal.getFitnessScore() << endl;
        relocal_tune = regis_relocal.getFinalTransformation();
        converged = regis_relocal.hasConverged();
        fitnessScore = regis_relocal.getFitnessScore();
        std::string anything;
        // cin >> anything;
        // visualize_pcd(pclCloud, gtmapCloud, unused_result);
    }
    else if (regis_type == "ndt")
    {
        pcl::NormalDistributionsTransform<PointT, PointT> regis_relocal;
        // Setting scale dependent NDT parameters
        // Setting minimum transformation difference for termination condition.
        regis_relocal.setTransformationEpsilon(0.01);
        // Setting maximum step size for More-Thuente line search.
        regis_relocal.setStepSize(step_size);
        //Setting Resolution of NDT grid structure (VoxelGridCovariance).
        regis_relocal.setResolution(reso);
        // Setting max number of registration iterations.
        regis_relocal.setMaximumIterations(max_iter);
        regis_relocal.setInputSource(pclCloud);
        // regis_relocal.setInputSource(pcl2DCloud);
        regis_relocal.setInputTarget(gtmapCloud);
        // regis_relocal.align(*unused_result);
        regis_relocal.align(*unused_result, regisInitialMat); // PCL regis non-eye initial is bad ... don't use (LeGO LOAM author also said pcl transform is weird.)
        lastregisMat = regis_relocal.getFinalTransformation().matrix();
        cout << "[indoor_localize] regis fit score: " << regis_relocal.getFitnessScore() << endl;
        relocal_tune = regis_relocal.getFinalTransformation();
        converged = regis_relocal.hasConverged();
        fitnessScore = regis_relocal.getFitnessScore();
    }
    t_localize.toc("localize");
    cout << "[indoor_localize] The regis.trans is \n"
            << relocal_tune.matrix() << endl;
    pcl::getTranslationAndEulerAngles(relocal_tune, x, y, z, roll, pitch, yaw);
    cout << "[indoor_localize] The result is "
            << "\n\tx:    " << x << "\ty:   " << y << "\tz:     " << z
            << "\n\troll: " << roll << "\tpitch: " << pitch << "\tyaw: " << yaw << endl;
    if (converged == false || fitnessScore > 0.2)
    {
        perror("regis_relocal is bad");
        cout << "[indoor_localize] warning: regis_relocal.hasConverged(): " << converged << endl;
        cout << "[indoor_localize] warning: regis fit score : " << fitnessScore << endl;
    }

    {
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*gtmapCloud, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time().fromSec(laserCloudRawTime);
        cloudMsgTemp.header.frame_id = "/velodyne";
        pubRelocalTarget.publish(cloudMsgTemp);
    }
    {
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*pcl2DCloud, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time().fromSec(laserCloudRawTime);
        cloudMsgTemp.header.frame_id = "/velodyne";
        pubRelocalSource.publish(cloudMsgTemp);
    }
    {
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*unused_result, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time().fromSec(laserCloudRawTime);
        cloudMsgTemp.header.frame_id = "/velodyne";
        pubRelocalregis.publish(cloudMsgTemp);
    }
    // //保存点云
    // std::string filename;
    // filename = "/home/ubuwgb/";
    // filename += std::to_string(laserCloudRawTime);
    // filename += ".pcd";
    // std::cout << "filename: " << filename << std::endl;
    // pcl::io::savePCDFileASCII(filename, *pclCloud);
}

void addPillar(PointCloud::Ptr map, float width, float start, float end)
{
    PointType p_pillar;
    p_pillar.x = 0;
    p_pillar.y = 0;
    p_pillar.z = 0;
    // 1
    p_pillar.x = start;
    for (float lateral = width; lateral <= width + 0.38; lateral = lateral + 0.1)
    {
        p_pillar.y = lateral;
        gtmapCloud->push_back(p_pillar);
    }
    // 2
    p_pillar.x = end;
    for (float lateral = width; lateral <= width + 0.38; lateral = lateral + 0.1)
    {
        p_pillar.y = lateral;
        gtmapCloud->push_back(p_pillar);
    }
    // 2
    p_pillar.y = width;
    for (float longi = start; longi <= end; longi = longi + 0.1)
    {
        p_pillar.x = longi;
        gtmapCloud->push_back(p_pillar);
    }
}

int main(int argc, char **argv)
{
    cout.setf(std::ios::fixed, std::ios::floatfield);
    ros::init(argc, argv, "bag2pcd");
    ros::NodeHandle nh;
    std::string input_topic;
    nh.getParam("input_topic", input_topic);
    nh.getParam("x", x);
    nh.getParam("y", y);
    nh.getParam("z", z);
    nh.getParam("roll", roll);
    nh.getParam("pitch", pitch);
    nh.getParam("yaw", yaw);
    nh.getParam("base_height", base_height);

    nh.getParam("max_iter", max_iter);
    nh.getParam("regis_type", regis_type);
    nh.getParam("translation_epsilon", translation_epsilon);
    nh.getParam("step_size", step_size);
    nh.getParam("reso", reso);
    nh.getParam("map_list", map_list);
    firstmsg = true;

    pubRelocalTarget = nh.advertise<sensor_msgs::PointCloud2>("/relocal_target_indoor", 2);
    pubRelocalSource = nh.advertise<sensor_msgs::PointCloud2>("/relocal_source_indoor", 2);
    pubRelocalregis = nh.advertise<sensor_msgs::PointCloud2>("/relocal_regis", 2);
    // ros::Subscriber pc = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 1000, CloudHandler);
    ros::Subscriber pc = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 1000, savetxtCloudHandler);

    float width = 2.64;
    PointType p_gtmap;
    // 1. width two points
    p_gtmap.x = 0;
    p_gtmap.y = 0;
    p_gtmap.z = 0;
    gtmapCloud->push_back(p_gtmap);
    p_gtmap.y = width;
    gtmapCloud->push_back(p_gtmap);
    // 2. longitudinal
    for (float wall = 14.201; wall <= 15.831; wall = wall + 0.1)
    {
        p_gtmap.x = wall;
        gtmapCloud->push_back(p_gtmap);
    }
    p_gtmap.y = 0;
    for (float wall = 0; wall <= 60; wall = wall + 0.1)
    {
        p_gtmap.x = wall;
        gtmapCloud->push_back(p_gtmap);
    }
    // 3. lateral
    p_gtmap.x = 0;
    p_gtmap.y = 0;
    p_gtmap.z = 0;
    for (float lateral = width; lateral <= width + 2; lateral = lateral + 0.1)
    {
        p_gtmap.y = lateral;
        gtmapCloud->push_back(p_gtmap);
    }
    p_gtmap.x = 14.201;
    for (float lateral = width; lateral <= width + 2; lateral = lateral + 0.1)
    {
        p_gtmap.y = lateral;
        if (lateral > width + 0.25)
        {
            p_gtmap.x = 14.201 + 0.11;
        }
        gtmapCloud->push_back(p_gtmap);
    }
    p_gtmap.x = 14.821;
    for (float lateral = width; lateral <= width + 2; lateral = lateral + 0.1)
    {
        p_gtmap.y = lateral;
        gtmapCloud->push_back(p_gtmap);
    }
    p_gtmap.x = 15.171;
    for (float lateral = width; lateral <= width + 2; lateral = lateral + 0.1)
    {
        p_gtmap.y = lateral;
        gtmapCloud->push_back(p_gtmap);
    }
    p_gtmap.x = 15.831;
    for (float lateral = width; lateral <= width + 2; lateral = lateral + 0.1)
    {
        p_gtmap.y = lateral;
        if (lateral > width + 0.25)
        {
            p_gtmap.x = 15.831 - 0.18;
        }
        gtmapCloud->push_back(p_gtmap);
    }
    // 4. add pillar
    // addPillar(gtmapCloud, width, 6.790);
    // addPillar(gtmapCloud, width, 22.996);
    scManager.makeAndSaveScancontextAndKeys(*gtmapCloud);

    frameIndex = 0;
    while (ros::ok())
    {
        ros::spin();
    }

    return 0;
}