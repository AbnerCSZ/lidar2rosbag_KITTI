#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>

#include<algorithm>
#include<chrono>
#include<sstream>
#include<string>
 
#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>

#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

ros::Publisher pubLaserCloud;
std::vector<std::string> file_lists;
std::vector<double> times_lists;

using namespace pcl;
using namespace std;

namespace po = boost::program_options;

void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL){
        std::string tmp_file = ptr->d_name;
        //std::cout << tmp_file <<endl;
        if (tmp_file[0] == '.')continue;
        if (type.size() <= 0){
            out_filelsits.push_back(ptr->d_name);
        }else{
            if (tmp_file.size() < type.size())continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
            if (tmp_cut_type == type){
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
}

bool computePairNum(std::string pair1,std::string pair2)
{
    return pair1 < pair2;
}

void sort_filelists(std::vector<std::string>& filists,std::string type)
{
    if (filists.empty())return;

    std::sort(filists.begin(),filists.end(),computePairNum);
}

void Benchmark2TUM(const string &num)
{
    cout << endl << "Reading KITTI benchmark from " << num << " ..." << endl;

    ifstream f;
    std::string in_path = "/data/Test/poses/00.txt";
    f.open(num.c_str());

    ifstream ft;
    std::string times_path = "/data/KITTI/dataset/sequences/"+num + "/times.txt";
    ft.open(num.c_str());

    ofstream fo;
    std::string out_path = "/data/Test/posesTUM/" +num + ".txt";
    fo.open(out_path.c_str());

        std::cout<<"in: "<<in_path<<"\ntime:  "<< times_path <<"\nout:"<<out_path<<std::endl;

    std::vector<double> t_lists;
    if(ft.is_open())
    {
        double time = 0.0f;
        while(!ft.eof() )
        {
            
            ft >>setprecision(12)>> time;
            t_lists.push_back(time);
        }
    }
    else{
        printf("no times file\n");
    }
    ft.close();
     
    if(f.is_open()  )
    {
        int i=0;
    	while(!f.eof())
    	{
    		string s;
    		getline(f,s);
    		if(!s.empty())
    		{
    			stringstream ss;
    			ss << s;
    			Eigen::Matrix<double,4,4> TwI;
    			double a,b,c,d,  e,f,g,h,   i,j,k,l;
    			ss>>a;  ss>>b;  ss>>c;  ss>>d;
    			ss>>e;  ss>>f;  ss>>g;  ss>>h;
    			ss>>i;  ss>>j;  ss>>k;  ss>>l;
    			TwI << a, b, c, d,
    				   e, f, g, h,
    				   i, j, k, l,
    				   0.,0.,0.,1.;
    			Eigen::Quaterniond q(TwI.block<3,3>(0,0));
    	        Eigen::Vector3d t = TwI.block<3,1>(0,3);

                fo << setprecision(12) << t_lists[i]  << " " << t(0) << " " << t(1) << " " << t(2)
                     << " " << q.x()<< " " << q.y() << " " << q.z() << " " << q.w() << endl;
    		}
            i++;
    	}


    	f.close();	
    }
    else
    	printf("ERROR: can not find the benchmark file!\n");

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar2rosbag");
    ros::NodeHandle nh;

  //temp:process benchmark
//   for(int i=0;i<=10;i++)
//   {
//     std::string _num ;
//     if(i<10)
//         _num= "0" + std::to_string(i);
//     else
//         _num="10";
//     Benchmark2TUM(_num); // useless, have to pass full name
//     printf("process  %d, \n", i);
//   }

    if(argc<3)
    {
        printf("ERROR: Please follow the example: rosrun pkg node input num_output:\n  rosrun lidar2rosbag lidar2rosbag /data/KITTI/dataset/sequences/04/ 04 \n");
        return -2;

    }

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_pub", 2);

    std::string input_dir = argv[1];
    std::string output_dir = argv[2];
    std::string bin_path = input_dir + "velodyne/" ;//"/data/KITTI/dataset/sequences/04/velodyne/";
    std::string times_path = input_dir + "times.txt";

    //load times
    times_lists.clear();
    ifstream timeFile(times_path, std::ios::in);
  
    if(timeFile.is_open())
    {
        double time = 0.0f;
        while(!timeFile.eof() )
        {
            
            timeFile >>setprecision(12)>> time;
            times_lists.push_back(time);
        }
    }

    read_filelists( bin_path, file_lists, "bin" );
    sort_filelists( file_lists, "bin" );
    
    for(int i =0;i<file_lists.size();i++)
    {
        std::cout << file_lists[i]<<std::endl;
    }

    for(int i =0;i<times_lists.size();i++)
    {
        std::cout << times_lists[i]<<std::endl;
    }

    std::cout <<"size:  "<< file_lists.size()<<"    "<<times_lists.size()<<endl;
    
    rosbag::Bag bag;
    bag.open(output_dir + ".bag", rosbag::bagmode::Write);

    //load point cloud
    for(int iter=0;iter<file_lists.size();iter++)
    {
        std::string infile = bin_path + file_lists[iter];
        ifstream input(infile.c_str(), ios::in | ios::binary);
        if(!input.is_open() ){
            cerr << "Could not read file: " << infile << endl;
            return -1;
        }
        // pcl::PointCloud<PointXYZI>::Ptr points (new pcl::PointCloud<PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI> points;
        const size_t kMaxNumberOfPoints = 1e6;  // From the Readme of raw files.
        points.clear();
        points.reserve(kMaxNumberOfPoints);

        int i;
        for (i=0; input.is_open() && !input.eof(); i++) {
            PointXYZI point;
            
            input.read((char *) &point.x, 3*sizeof(float));
            input.read((char *) &point.intensity, sizeof(float));
            points.push_back(point);
        }
        input.close();

        ros::Time timestamp_ros(times_lists[iter]==0?  ros::TIME_MIN.toSec()  :times_lists[iter]);
       // timestampToRos(times_lists[iter], &timestamp_ros);

        points.header.stamp = times_lists[iter] ;
        points.header.frame_id = "velodyne";

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(points, output);
        output.header.stamp = timestamp_ros  ;
        output.header.frame_id = "velodyne";
        
        pubLaserCloud.publish(output);
        std::cout<<"ros time : "<< output.header.stamp.toSec() <<"  with  "<<timestamp_ros.toSec()<<endl;
        bag.write("velodyne_points", timestamp_ros, output);

    }

    printf("lidar 2 kitti rosbag done\n");


//  ros::Publisher pubBag = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);
//   rosbag::Bag bag;
//   bag.open("/data/KITTI/velodyne/04.bag", rosbag::bagmode::Read);

// //load lidar rosbag
//   for(rosbag::MessageInstance const m: rosbag::View(bag))
//   {
//     auto begin_time = std::chrono::system_clock::now();
//     sensor_msgs::PointCloud2::ConstPtr i = m.instantiate<sensor_msgs::PointCloud2>();
//      if (i != NULL)
//        pubBag.publish( *i );
//     sleep(1);
//   }

//   bag.close();

  //ros::spin();

  return 0;
}
