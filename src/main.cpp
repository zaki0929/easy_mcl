#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdlib>
#include <cmath>

struct Position{
  double x;
  double y;
  double th;
};

class GlobalMap{
public:
  GlobalMap();
  ~GlobalMap();
  Eigen::MatrixXd get_global_map();
  Eigen::MatrixXd binarize_map(Eigen::MatrixXd img_e);
  void export_map_image(Eigen::MatrixXd img_e);

private:
  std::string homepath;
};

class LocalMap{
public:
  LocalMap(ros::NodeHandle& nh);
  ~LocalMap();
  ros::Subscriber scan_sub;
  sensor_msgs::LaserScan::ConstPtr scan;
  Eigen::MatrixXd get_local_map();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void export_map_image(Eigen::MatrixXd img_e);
  const int size;
  int scan_toggle;

private:
  std::string homepath;
};

class Particle{
public:
  Particle(ros::NodeHandle& nh);
  ~Particle();
  ros::Subscriber odom_sub;
  nav_msgs::Odometry::ConstPtr odom;
  Eigen::MatrixXd get_path();
  void scanCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void export_map_image(Eigen::MatrixXd img_e);
  double weight;
  int odom_toggle;
  int size_x;
  int size_y;

private:
  std::string homepath;
  Eigen::MatrixXd path_map;
};

inline Eigen::Vector3d set_pose(double x, double y, double th){
  Eigen::Vector3d vec;
  vec(0) = x;
  vec(1) = y;
  vec(2) = th;
  return vec;
}
//----------------------------------------------------------------------

GlobalMap::GlobalMap(){
  homepath = std::getenv("HOME");
}

GlobalMap::~GlobalMap(){
  ROS_INFO("global map: successfully released memory");
}

Eigen::MatrixXd GlobalMap::get_global_map(){
  // 地図の読み込み
  cv::Mat img = cv::imread(homepath + "/catkin_ws/src/easy_mcl/map/map.pgm", 0);
  if(img.empty()){
    ROS_ERROR("global map: unable to open the map");
  }else{
    ROS_INFO("global map: map loaded");
  }
  
  // 地図データをOpenCVからEigenに渡す
  Eigen::MatrixXd img_e;
  cv::cv2eigen(img, img_e);
  ROS_INFO("global map: opencv -> eigen");
  
  // 地図データを二値化する
  img_e = binarize_map(img_e);

  return img_e;
}

Eigen::MatrixXd GlobalMap::binarize_map(Eigen::MatrixXd img_e){
  // 200を閾値にして地図を二値化する
  for(int i=0; i<img_e.rows(); i++){
    for(int j=0; j<img_e.cols(); j++){
      if(img_e(j, i) > 200){
        img_e(j, i) = 255;
      }
      else{
        img_e(j, i) = 0;
      }
    }
  }
  ROS_INFO("global map: binarized");
  return img_e;
}

void GlobalMap::export_map_image(Eigen::MatrixXd img_e){
  // 地図データをEigenからOpenCVに渡し、出力
  cv::Mat img;
  cv::eigen2cv(img_e, img);
  ROS_INFO("global map: eigen -> opencv");
  cv::imwrite(homepath + "/catkin_ws/src/easy_mcl/map/global_map.pgm", img);
  ROS_INFO("global map: map exported");
}

//----------------------------------------------------------------------

LocalMap::LocalMap(ros::NodeHandle& nh) : size(240){    // 240 x 240 
  scan_toggle = 0;
  homepath = std::getenv("HOME");
  scan_sub = nh.subscribe("scan", 1000, &LocalMap::scanCallback, this);
}

LocalMap::~LocalMap(){
  ROS_INFO("local map: successfully released memory");
}

void LocalMap::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(scan_toggle){
    //ROS_INFO("local map: get data from scan topic");
  }else{
    scan_toggle = 1;
  }
  scan = msg;
}

Eigen::MatrixXd LocalMap::get_local_map(){
  // 全ての要素の値が255でローカルマップサイズの行列を取得
  Eigen::MatrixXd img_e = Eigen::MatrixXd::Ones(size, size); 
  img_e *= 255;

  for(double th=scan->angle_min, i=0; th<=scan->angle_max; th+=scan->angle_increment, i++){
    if(scan->ranges[i] > 0){
      int x = int(scan->ranges[i]*std::cos(th+M_PI_2)/0.05);
      int y = int(scan->ranges[i]*std::sin(th+M_PI_2)/0.05);

      img_e(int(size/2)-y, int(size/2)+x) = 0;
    }
  }
  
  ROS_INFO("local map: completed writing map");
  return img_e;
}

void LocalMap::export_map_image(Eigen::MatrixXd img_e){
  // 地図データをEigenからOpenCVに渡し、出力
  cv::Mat img;
  cv::eigen2cv(img_e, img);
  ROS_INFO("local map: eigen -> opencv");
  cv::imwrite(homepath + "/catkin_ws/src/easy_mcl/map/local_map.pgm", img);
  ROS_INFO("local map: map exported");
}

//----------------------------------------------------------------------

Particle::Particle(ros::NodeHandle& nh){
  homepath = std::getenv("HOME");

  // 地図の読み込み
  cv::Mat img = cv::imread(homepath + "/catkin_ws/src/easy_mcl/map/map.pgm", 0);
  if(img.empty()){
    ROS_ERROR("particle: unable to open the map");
  }else{
    ROS_INFO("particle: map loaded");
  }
  size_x = img.rows;
  size_y = img.cols;
  ROS_INFO("x: %d, y: %d", size_x, size_y);


  odom_toggle = 0;
  odom_sub = nh.subscribe("odom", 1000, &Particle::scanCallback, this);

  weight = 0;

  // 軌跡をまっさらにする
  path_map = Eigen::MatrixXd::Ones(size_y, size_x); 
  path_map *= 255;
}

Particle::~Particle(){
  ROS_INFO("particle: successfully released memory");
}

void Particle::scanCallback(const nav_msgs::Odometry::ConstPtr& msg){
  if(odom_toggle){
    //ROS_INFO("particle: get data from odom topic");
  }else{
    odom_toggle = 1;
  }
  odom = msg;
}

Eigen::MatrixXd Particle::get_path(){
  Eigen::Vector3d odom_pos = set_pose(odom->pose.pose.position.x, odom->pose.pose.position.y, 0.0); 
  int x = odom_pos(0)/0.05;
  int y = odom_pos(1)/0.05;

  ROS_INFO("x: %d, y: %d", int(x), int(y));

  path_map(int(size_y/2)-y, int(size_x/2)+x) = 0;

  return path_map;
}

void Particle::export_map_image(Eigen::MatrixXd img_e){
  // 地図データをEigenからOpenCVに渡し、出力
  cv::Mat img;
  cv::eigen2cv(img_e, img);
  ROS_INFO("particle: eigen -> opencv");
  cv::imwrite(homepath + "/catkin_ws/src/easy_mcl/map/path_map.pgm", img);
  ROS_INFO("particle: map exported");
}

//----------------------------------------------------------------------

int main(int argc, char** argv){
  ros::init(argc, argv, "easy_mcl");
  ros::NodeHandle nh;
  ros::Rate rate(0.5);

  GlobalMap *gm = new GlobalMap();
  LocalMap *lm = new LocalMap(nh);
  Particle *p = new Particle(nh);

  while(ros::ok()){

    if(lm->scan_toggle){
      // 地図データの読み込み
      Eigen::MatrixXd global_map = gm->get_global_map();
      Eigen::MatrixXd local_map = lm->get_local_map();

      // 地図の書き出し
      gm->export_map_image(global_map);
      lm->export_map_image(local_map);
    }else{
      ROS_INFO("waiting scan topic...");
    }

    if(p->odom_toggle){
      Eigen::MatrixXd path_map = p->get_path();
      p->export_map_image(path_map);
    }else{
      ROS_INFO("waiting odom topic...");
    }

    ros::spinOnce();
    rate.sleep();
  }
    
  delete gm;
  delete lm;
  delete p;

  return 0;
}
