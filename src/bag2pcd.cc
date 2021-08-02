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

#include "bag2pcd/Scancontext.cpp"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h> //error: ‘pcl::visualization’ has not been declared

using std::cout;
using std::endl;

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
PointCloud::Ptr mapCloud(new PointCloud);
Eigen::Matrix4f lastregisMat;
bool firstmsg;
SCManager scManager;
ros::Publisher pubRelocalTarget, pubRelocalSource, pubRelocalregis;
double YawDiff;
std::vector<gtdata> mapgt;
int findGT_plus;
std::ofstream error;
pcl::VoxelGrid<PointT> downSizeFilter;
bool downsizeFlag;
double downsize, zLow, zHigh;

// registration params
int max_iter;
double translation_epsilon;
double step_size, reso;
std::string regis_type;
bool converged;
double fitnessScore;

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

void find_gt(double pctime, double &gt_x, double &gt_y)
{
    if (!findGT_plus)
    {
        for (int i = 0; i < mapgt.size() - 1; i++)
        {
            if (fabs(mapgt[i].time - pctime) < fabs(mapgt[i + 1].time - pctime))
            {
                // cout << i << "th diff0!: \t" << fabs(mapgt[i].time - pctime) << endl;
                // cout << i + 1 << "th diff0!: \t" << fabs(mapgt[i + 1].time - pctime) << endl;
                findGT_plus = i;
                break;
            }
        }
    }
    else
    {
        for (int i = findGT_plus; i < mapgt.size() - 1; i++)
        {
            if (fabs(mapgt[i].time - pctime) < fabs(mapgt[i + 1].time - pctime))
            {
                // cout << i << "th diff: \t" << fabs(mapgt[i].time - pctime) << endl;
                // cout << i + 1 << "th diff: \t" << fabs(mapgt[i + 1].time - pctime) << endl;
                findGT_plus = i;
                break;
            }
        }
    }
    gt_x = mapgt[findGT_plus].x;
    gt_y = mapgt[findGT_plus].y;

    if (fabs(mapgt[findGT_plus].time - pctime) > 0.2)
    {
        cout << "[find_gt] warning: fabs(mapgt[" << findGT_plus << "].time - pctime): \t"
                << fabs(mapgt[findGT_plus].time - pctime) << endl;
        cout << "[find_gt] warning: pctime: \t" << pctime << endl;
        cout << "[find_gt] warning: gttime: \t" << mapgt[findGT_plus].time << endl;
        perror("[find_gt] time diff too large");
        // exit(1);
    }
}

void CloudHandler(const sensor_msgs::PointCloud2ConstPtr &cloudmsg)
{
    TicToc t_localize(true);
    double laserCloudRawTime = cloudmsg->header.stamp.toSec();

    pclCloud->clear();
    PointCloud::Ptr oripclCloud(new PointCloud);
    pcl::fromROSMsg(*cloudmsg, *oripclCloud);

    PointCloud::Ptr pclCloudori2(new PointCloud);
	pcl::PassThrough<PointT> pass; // 声明直通滤波
	pass.setInputCloud(oripclCloud); // 传入点云数据
	pass.setFilterFieldName("z"); // 设置操作的坐标轴
	pass.setFilterLimits(zLow, zHigh); // 设置坐标范围
	// pass.setFilterLimitsNegative(true); // 保留数据函数
	pass.filter(*pclCloudori2);  // 进行滤波输出
    if (downsizeFlag)
    {
        downSizeFilter.setInputCloud(pclCloudori2);
        downSizeFilter.filter(*pclCloud);
    }

    // for (int i = 0; i < pclCloud->points.size(); i++)
    // {
    //     double x = -pclCloud->points[i].x;
    //     double y = -pclCloud->points[i].z;
    //     double z = -pclCloud->points[i].y;

    //     if (z > -0.25 && sqrt(x * x + y * y) < 10)
    //     {
    //         double gridx = double(int(x * 20));
    //         double gridy = double(int(y * 20));
    //         PointT onepoint;
    //         onepoint.x = gridx;
    //         onepoint.y = gridy;
    //         onepoint.z = 0;
    //         pcl2DCloud->points.push_back(onepoint);
    //     }
    // }

    // Align clouds
    // Eigen::Affine3f regisInitialMatFoo = pcl::getTransformation(0, 0, 0, 0, 0, -YawDiff); // because within cam coord: (z, x, y, yaw, roll, pitch)
    Eigen::Matrix4f regisInitialMat, relocal_tune;
    if (firstmsg)
    {
        cout << "1 YawDiff: " << YawDiff << endl;
        cout << "1 fabs(YawDiff): " << fabs(YawDiff) << endl;
        cout << "1 fabs(YawDiff) < 1e-6: " << (fabs(YawDiff) < 1e-6) << endl;
        if (fabs(YawDiff) < 1e-6)
        {
            auto localizeResult = scManager.detectRelocalID(*pclCloud);
            YawDiff = localizeResult.second;
            cout << "2 YawDiff: " << YawDiff << endl;
        }
        cout << "3 YawDiff: " << YawDiff << endl;
        Eigen::Affine3f regisInitialMatFoo = pcl::getTransformation(0, 0, 0, 0, 0, YawDiff);
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
        regis_relocal.setMaxCorrespondenceDistance(3);
        regis_relocal.setMaximumIterations(max_iter);
        regis_relocal.setTransformationEpsilon(translation_epsilon*translation_epsilon);
        regis_relocal.setEuclideanFitnessEpsilon(1e-2);
        regis_relocal.setRANSACIterations(0);
        regis_relocal.setInputSource(pclCloud);
        // regis_relocal.setInputSource(pcl2DCloud);
        regis_relocal.setInputTarget(mapCloud);
        // regis_relocal.align(*unused_result);
        regis_relocal.align(*unused_result, regisInitialMat); // PCL regis non-eye initial is bad ... don't use (LeGO LOAM author also said pcl transform is weird.)
        lastregisMat = regis_relocal.getFinalTransformation().matrix();
        cout << "[indoor_localize] regis fit score: " << regis_relocal.getFitnessScore() << endl;
        relocal_tune = regis_relocal.getFinalTransformation();
        converged = regis_relocal.hasConverged();
        fitnessScore = regis_relocal.getFitnessScore();
        std::string anything;
        // cin >> anything;
        // visualize_pcd(pclCloud, mapCloud, unused_result);
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
        regis_relocal.setInputTarget(mapCloud);
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
    if (converged == false || fitnessScore > 0.2)
    {
        perror("regis_relocal is bad");
        cout << "[indoor_localize] warning: regis_relocal.hasConverged(): " << converged << endl;
        cout << "[indoor_localize] warning: regis fit score : " << fitnessScore << endl;
    }
    else
    {
        Eigen::Matrix4f result = relocal_tune.matrix();
        double gt_x, gt_y;
        find_gt(laserCloudRawTime, gt_x, gt_y);
        error << std::setprecision(15)
              << laserCloudRawTime
              << " " << gt_x - result(1, 3)
              << " " << -gt_y - result(0, 3)
              << " " << t_localize.time() << "\n";
    }
    {
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*mapCloud, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time().fromSec(laserCloudRawTime);
        cloudMsgTemp.header.frame_id = "/velodyne";
        pubRelocalTarget.publish(cloudMsgTemp);
    }
    {
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*pclCloud, cloudMsgTemp);
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

int main(int argc, char **argv)
{
    cout.setf(std::ios::fixed, std::ios::floatfield);
    ros::init(argc, argv, "bag2pcd");
    ros::NodeHandle nh;
    std::string input_topic;
    nh.getParam("input_topic", input_topic);
    nh.getParam("YawDiff", YawDiff);
    nh.getParam("downsizeFlag", downsizeFlag);
    nh.getParam("downsize", downsize);
    nh.getParam("zLow", zLow);
    nh.getParam("zHigh", zHigh);
    nh.getParam("max_iter", max_iter);
    nh.getParam("translation_epsilon", translation_epsilon);
    nh.getParam("reso", reso);
    nh.getParam("step_size", step_size);
    nh.getParam("regis_type", regis_type);
    firstmsg = true;
    findGT_plus = 0;
    downSizeFilter.setLeafSize(downsize, downsize, downsize);

    pubRelocalTarget = nh.advertise<sensor_msgs::PointCloud2>("/relocal_target_indoor", 2);
    pubRelocalSource = nh.advertise<sensor_msgs::PointCloud2>("/relocal_source_indoor", 2);
    pubRelocalregis = nh.advertise<sensor_msgs::PointCloud2>("/relocal_regis", 2);
    
    ros::Subscriber pc = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 1000, CloudHandler);
    //加载点云文件
    PointCloud::Ptr mapCloudori(new PointCloud);
    PointCloud::Ptr mapCloudori2(new PointCloud);
    pcl::io::loadPCDFile("/media/ubuwgb/SSS_X64FRE_/data_wgb/smallcar/TX2/map.pcd", *mapCloudori);
	pcl::PassThrough<PointT> pass; // 声明直通滤波
	pass.setInputCloud(mapCloudori); // 传入点云数据
	pass.setFilterFieldName("z"); // 设置操作的坐标轴
	pass.setFilterLimits(zLow, zHigh); // 设置坐标范围
	// pass.setFilterLimitsNegative(true); // 保留数据函数
	pass.filter(*mapCloudori2);  // 进行滤波输出
    if (downsizeFlag)
    {
        downSizeFilter.setInputCloud(mapCloudori2);
        downSizeFilter.filter(*mapCloud);
    }
    scManager.makeAndSaveScancontextAndKeys(*mapCloud);

    std::ifstream loadMap;
    loadMap.open("/home/ubuwgb/Downloads/convert_png/build/data.txt");
    // cout << "loadMap.eof(): " << loadMap.eof() << endl;
    // cout << "loadMap.good(): " << loadMap.good() << endl;
    double gridy = 0;
    double gridx = 0;
    while (!loadMap.eof() && loadMap.good())
    {
        char buf[1000];
        loadMap.getline(buf, 1000);
        sscanf(buf, "%lf,%lf\n", &gridy, &gridx);
        // cout << "gridy: " << gridy
        //      << ", gridx: " << gridx << endl;
        // data[gridy * w + gridx] = 0;
        PointT onepoint;
        onepoint.x = gridx;
        onepoint.x = 600 - gridy;
        onepoint.z = 0;
        // mapCloud->points.push_back(onepoint);
    }
    loadMap.close();

    std::ifstream loadMapGT;
    loadMapGT.open("/media/ubuwgb/SSS_X64FRE_/data_wgb/smallcar/GTfromlego.txt");
    // cout << "loadMapGT.eof(): " << loadMapGT.eof() << endl;
    // cout << "loadMapGT.good(): " << loadMapGT.good() << endl;
    double time, gtz, gtx;
    while (!loadMapGT.eof() && loadMapGT.good())
    {
        char buf[1000];
        gtdata onegt;
        loadMapGT.getline(buf, 1000);
        sscanf(buf, "%lf;%lf;%lf\n", &time, &gtz, &gtx);
        // cout << "time: " << time
        //      << ", gtz: " << gtz
        //      << ", gtx: " << gtx << endl;
        onegt.time = time - 83787.16;
        onegt.x = gtz;
        onegt.y = gtx;
        mapgt.emplace_back(onegt);
    }
    loadMapGT.close();
    cout << mapgt.size() << " position is loaded." << endl;
    error.open("/home/ubuwgb/github/lidar-imu-calibration/indoor_car/error.txt", std::ios::app);
    while(ros::ok())
    {
        ros::spin();
    }
    error.close();

    return 0;
}