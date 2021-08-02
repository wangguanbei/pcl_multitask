#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <fstream>

#include <Eigen/Dense>
// #include <Eigen/Geometry>
#include <pcl/common/centroid.h> //computeMeanAndCovarianceMatrix
#include <Eigen/src/Core/util/Constants.h>

// struct PointXYZIRT
// {
//     PCL_ADD_POINT4D
//     float intensity;
//     uint16_t ring;
//     float time;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
// } EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
//                                   (float, x, x)
//                                   (float, y, y)
//                                   (float, z, z)
//                                   (float, intensity, intensity)
//                                   (uint16_t, ring, ring)
//                                   (float, time, time)
// )

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloudT;

Eigen::Matrix4f CreateRotateMatrix(Eigen::Vector3f before,Eigen::Vector3f after)
{
    before.normalize();
    after.normalize();
 
    float angle = acos(before.dot(after));
    Eigen::Vector3f p_rotate =before.cross(after);
    p_rotate.normalize();
 
    Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
    rotationMatrix(0, 0) = cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
    rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle) - p_rotate[2] * sin(angle));//这里跟公式比多了一个括号，但是看实验结果它是对的。
    rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
 
 
    rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
 
 
    rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) +p_rotate[0] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));
 
    return rotationMatrix;
}

void estimate_plane_(PointCloudT::Ptr g_ground_pc/*, double &roll, double &pitch, double &yaw, double &offset_z*/)
{
    Eigen::MatrixXf normal_;
    // Create covarian matrix in single pass.
    // TODO: compare the efficiency.
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean; // 归一化坐标值
    // computeMeanAndCovarianceMatrix主要是PCA过程中计算平均值和协方差矩阵 ,对地面点(最小的n个点)进行计算协方差和平均值
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);
    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::ComputeFullU);
    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2)); // 取最小的特征值对应的特征向量作为法向量
    std::cout << "normal_:\n"<< normal_ << std::endl;  //45 -0 0
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>(); // seeds_mean 地面点的平均值

    // according to normal.T*[x,y,z] = -d
    // offset_z = -(normal_.transpose()*seeds_mean)(0,0); // 计算d   D=d
    // set distance threhold to `th_dist - d`
    //th_dist_d_ = th_dist_ - offset_z;// 这里只考虑在拟合的平面上方的点 小于这个范围的点当做地面
    
    // return the equation parameters
    
    Eigen::MatrixXf vectorAfter = Eigen::MatrixXf::Zero(3, 1);
    vectorAfter << 0.0, 0.0, 1.0;
    Eigen::Matrix4f rotationMatrix = CreateRotateMatrix(normal_, vectorAfter);
    std::cout << "4*4 rotationMatrix:\n"<< rotationMatrix << std::endl;
    
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix<<   rotationMatrix(0, 0), rotationMatrix(0, 1), rotationMatrix(0, 2),
                        rotationMatrix(1, 0), rotationMatrix(1, 1), rotationMatrix(1, 2),
                        rotationMatrix(2, 0), rotationMatrix(2, 1), rotationMatrix(2, 2);
    std::cout << "3*3 rotation_matrix:\n";
    std::cout << rotationMatrix(0, 0) << "," << rotationMatrix(0, 1) << "," << rotationMatrix(0, 2) << std::endl;
    std::cout << rotationMatrix(1, 0) << "," << rotationMatrix(1, 1) << "," << rotationMatrix(1, 2) << std::endl;
    std::cout << rotationMatrix(2, 0) << "," << rotationMatrix(2, 1) << "," << rotationMatrix(2, 2) << std::endl;

    // 2,1,0;
    Eigen::Vector3d eulerAngle = rotation_matrix.eulerAngles(0,1,2);
    std::cout << "eulerAngle:\n"<< eulerAngle << std::endl;
    // yaw = eulerAngle(2);
    // roll = eulerAngle(1);
    // pitch = eulerAngle(0);
 
          
    // yaw = fmod( ( yaw + 2*M_PI),  (2*M_PI) ) / M_PI * 180;
    // roll = fmod( ( roll + 2*M_PI),  (2*M_PI) ) / M_PI * 180;
    // pitch = fmod( ( pitch + 2*M_PI),  (2*M_PI) ) / M_PI * 180;
    
    /*
	std::cout << "yaw:"<< yaw << ", roll:" << roll << ", pitch:" << pitch << std::endl;  //45 -0 0
	std::cout << "offset_z:"<< offset_z << std::endl;  //45 -0 0
	cout<<  rotationMatrix(0, 0) <<","<< rotationMatrix(0, 1) <<","<< rotationMatrix(0, 2) <<","<< rotationMatrix(0, 3)<< endl; 
    cout<<  rotationMatrix(1, 0) <<","<< rotationMatrix(1, 1) <<","<< rotationMatrix(1, 2) <<","<< rotationMatrix(1, 3)<< endl;
    cout<<  rotationMatrix(2, 0) <<","<< rotationMatrix(2, 1) <<","<< rotationMatrix(2, 2) <<","<< rotationMatrix(2, 3)<< endl; 
    cout<<  rotationMatrix(3, 0) <<","<< rotationMatrix(3, 1) <<","<< rotationMatrix(3, 2) <<","<< rotationMatrix(3, 3)<< endl; 
    */
}// estimate_plane_

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calib_lidar2car_byground");
    ros::NodeHandle nh;
    std::string input_path;
    nh.getParam("input_path", input_path);
    PointCloudT::Ptr ground(new PointCloudT);

    //加载点云文件
    pcl::io::loadPCDFile(input_path.c_str(), *ground);

    estimate_plane_(ground);

    return 0;
}