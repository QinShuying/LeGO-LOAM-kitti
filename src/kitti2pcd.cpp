
#include "fstream"
#include "iostream"
#include <ctime>
#include <dirent.h>
#include "ros/ros.h"

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>


//#include "fcn_data_gen/ground_remove.h"
using namespace std;
typedef pcl::PointXYZI PointType;

static ros::Publisher g_cloud_pub;
static std::vector<std::string> file_lists;
vector<string> binfiles;


bool exists_file(const std::string &name) {
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

void Load_binfile() {
    cout << "Start Load" << endl;

    string strPath = "/media/qsy-5208/Resource/SLAM_DATA/Datasets/KITTI_odometry/data_odometry_velodyne/sequences/1-10/01/velodyne";    // 109 + 6 +4

    int img_i = -1;
    do {
        img_i = img_i + 1;

        stringstream ss;
        ss << setfill('0') << setw(6) << img_i;
        std::string file = strPath + "/" + ss.str() + ".bin";
        if (exists_file(file)) {
            double t = img_i / 10.0;
            ss.clear();
            ss.str("");
            ss << setfill('0') << setw(6) << img_i;
            binfiles.push_back(strPath + "/" + ss.str() + ".bin");
        } else
            break;
    } while (1);

    cout << "binfiles.size(): " << binfiles.size() << endl;
}

void readKittiPclBinData(std::string &in_file, std::string& out_file)
{
    // load point cloud
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<PointType>::Ptr points (new pcl::PointCloud<PointType>);
    sensor_msgs::PointCloud2 outputs;

    int i;
    for (i=0; input.good() && !input.eof(); i++) {
        PointType point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        points->push_back(point);
    }
    input.close();

//    g_cloud_pub.publish(points);


//    Save pcl outputs
//    std::cout << "Read KTTI point cloud with " << i << " points, writing to " << out_file << std::endl;
//    pcl::PCDWriter writer;
//    writer.write< PointType > (out_file, *points, false);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kitti2pcd");
    ros::NodeHandle nh("~");

    std::string path;
    ROS_INFO("ROS Started.");
    g_cloud_pub = nh.advertise< pcl::PointCloud< PointType > > ("point_chatter", 1);

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_pub", 10);


    Load_binfile();
    std::string pcd_path = "/home/qsy-5208/Documents/LeGO/LeGO-LOAM-kitti/kitti_pcd_01/";
    ros::Rate rate(200);
    int i = 0;
    while (ros::ok())
    {
        std::string bin_file = binfiles[i];
        std::string tmp_str = binfiles[i].substr(109, binfiles[i].length() - 109 - 4) + ".pcd";
        std::string pcd_file = pcd_path + tmp_str;

        pcl::PointCloud<PointType> cloud;
        pcl::io::loadPCDFile(pcd_file, cloud);
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud, output);// 转换成ROS下的数据类型 最终通过topic发布
        output.header.stamp = ros::Time::now();
        output.header.frame_id = "slam";
        pcl_pub.publish(output);
        rate.sleep();
        i++;
        if(i==binfiles.size())
            break;
    }
    return 0;
}
