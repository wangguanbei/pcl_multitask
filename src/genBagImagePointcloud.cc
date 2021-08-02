#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h> //error: ‘fromROSMsg’ is not a member of ‘pcl’
#include <math.h>                            //PI
#include <vector>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


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

typedef PointXYZIRT PointT;
// typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//some global variable
cv_bridge::CvImage bridge;
ros::Publisher pubPointcloud, pubImage;
std::vector<double> image_timelist;
std::string imagePath;
std::vector<std::string> imagefiles;
int find_plus;

int findIndex(std::string ori, /*std::string b, */std::string c)
{
    int index_f, index_l;
    int index_gap;
    // index_f = a.find(b) + 4;
    index_f = 0;
    index_l = ori.find(c);
    index_gap = index_l - index_f;
    return atoi((ori.substr(index_f, index_gap)).c_str());
}
int cmp(std::string a, std::string b)
{
    int c, d;
    // c = findIndex(a, "Coin", ".bin");
    // d = findIndex(b, "Coin", ".bin");
    c = findIndex(a, "th_keyframe");
    d = findIndex(b, "th_keyframe");
    return  c < d;
}
std::vector<std::string> getFiles(std::string cate_dir)
{
    std::vector<std::string> files; //存放文件名
    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if ((dir = opendir(cate_dir.c_str())) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr = readdir(dir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) ///current dir OR parrent dir
            continue;
        else if (ptr->d_type == 8) ///file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            files.push_back(ptr->d_name);
        else if (ptr->d_type == 10) ///link file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            continue;
        else if (ptr->d_type == 4) ///dir
        {
            files.push_back(ptr->d_name);
            /*
		        memset(base,'\0',sizeof(base));
		        strcpy(base,basePath);
		        strcat(base,"/");
		        strcat(base,ptr->d_nSame);
		        readFileList(base);
			*/
        }
    }
    closedir(dir);

    //排序，按从小到大排序
    sort(files.begin(), files.end(), cmp);
    return files;
}


void CloudHandler(const sensor_msgs::PointCloud2ConstPtr &cloudmsg)
{
    std::string oneimagefile;
    double pctime = cloudmsg->header.stamp.toSec() * 1000;
    if (!find_plus)
    {
        for (int i = 0; i < image_timelist.size() - 1; i++)
        {
            if (abs(image_timelist[i] - pctime) < abs(image_timelist[i + 1] - pctime))
            {
                // cout << i << "th diff0!: \t" << abs(image_timelist[i] - pctime) << endl;
                // cout << i + 1 << "th diff0!: \t" << abs(image_timelist[i + 1] - pctime) << endl;
                oneimagefile = imagePath + imagefiles[i];
                find_plus = i;
                break;
            }
        }
    }
    else
    {
        for (int i = find_plus; i < image_timelist.size() - 1; i++)
        {
            if (abs(image_timelist[i] - pctime) < abs(image_timelist[i + 1] - pctime))
            {
                // cout << i << "th diff: \t" << abs(image_timelist[i] - pctime) << endl;
                // cout << i + 1 << "th diff: \t" << abs(image_timelist[i + 1] - pctime) << endl;
                oneimagefile = imagePath + imagefiles[i];
                find_plus = i;
                break;
            }
        }
    }
    if (abs(image_timelist[find_plus] - pctime) > 5)
    {
        cout << "pctime: \t" << pctime << endl;
        cout << "imagetime: \t" << image_timelist[find_plus] << endl;
    }
    cv::Mat oneimage = cv::imread(oneimagefile.c_str(), cv::IMREAD_GRAYSCALE);
    bridge.image = oneimage;
    bridge.encoding = "mono8";
    sensor_msgs::Image::Ptr imagemsg = bridge.toImageMsg();

    sensor_msgs::PointCloud2 cloudMsgTemp = *cloudmsg;
    cloudMsgTemp.header.stamp = ros::Time::now();
    cloudMsgTemp.header.frame_id = "/velodyne";
    imagemsg->header.stamp = cloudMsgTemp.header.stamp;

    pubImage.publish(imagemsg);
    pubPointcloud.publish(cloudMsgTemp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "genBagImagePointcloud");
    ros::NodeHandle nh;
    std::string pc_input_topic;
    int imageNum;
    nh.getParam("pc_input_topic", pc_input_topic);
    nh.getParam("imagePath", imagePath);
    nh.getParam("imageNum", imageNum);

    ros::Subscriber pc_sub = nh.subscribe<sensor_msgs::PointCloud2>(pc_input_topic, 5, CloudHandler);
    
    pubPointcloud = nh.advertise<sensor_msgs::PointCloud2>("/sync_pointcloud", 2);
    pubImage = nh.advertise<sensor_msgs::Image>("/sync_image", 2);
    find_plus = 0;

    std::ifstream loadImageInfo;
    double time, nothing;
    std::string imageInfoPath = imagePath + "cam_info.txt";
    loadImageInfo.open(imageInfoPath.c_str());
    cout.setf(std::ios::fixed, std::ios::floatfield);
    for (int i = 0; i < imageNum; i++)
    {
        char buf[1000];
        loadImageInfo.getline(buf, 1000);
        // fscanf(loadImageInfo, "%lf %lf\n", &nothing, &time);
        sscanf(buf, "%lf %lf\n", &nothing, &time);
        // cout << "loading time: " << time << endl;
        image_timelist.emplace_back(time);
    }
    loadImageInfo.close();
    imagePath = imagePath + "image_1/";
    imagefiles = getFiles(imagePath);

    // spin
    ros::spin();

    return 0;
}