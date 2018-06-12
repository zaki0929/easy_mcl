#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdlib>

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
  Eigen::MatrixXd get_local_map();
  void export_map_image(Eigen::MatrixXd img_e);
  ros::Subscriber scan_sub;
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  sensor_msgs::LaserScan::ConstPtr scan;
  const int size;
  int toggle;

private:
  std::string homepath;
};

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
        img_e(i, j) = 0;
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

LocalMap::LocalMap(ros::NodeHandle& nh) : size(240){    // 240 x 240 
  toggle = 0;
  homepath = std::getenv("HOME");
  scan_sub = nh.subscribe("scan", 1000, &LocalMap::scanCallback, this);
}

LocalMap::~LocalMap(){
  ROS_INFO("local map: successfully released memory");
}

void LocalMap::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  toggle = 1;
  ROS_INFO("local_map: get data from scan topic");
  scan = msg;
}

inline double null_check(double target){
  if(!(target > 0)){
      target = 5.6;
  }
  return target;
}

Eigen::MatrixXd LocalMap::get_local_map(){
  
  double center = null_check(scan->ranges[(-scan->angle_min)/scan->angle_increment]); 
  ROS_INFO("local map: %lf", center); 
  
  int center_cell = int(center/0.05);

  Eigen::MatrixXd img_e = Eigen::MatrixXd::Ones(size, size); 
  
  img_e *= 255;

  for(int i=0; i<img_e.rows(); i++){
    for(int j=0; j<img_e.cols(); j++){
      if(j == center_cell + int(size/2)){
        img_e(j, i) = 0;
      }
    }
  }
  
  ROS_INFO("global map: completed writing map");
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

int main(int argc, char** argv){
  ros::init(argc, argv, "easy_mcl");
  ros::NodeHandle nh;
  ros::Rate rate(1);

  GlobalMap *gm = new GlobalMap();
  LocalMap *lm = new LocalMap(nh);
  while(ros::ok()){

    if(lm->toggle){
      // 地図データの読み込み
      Eigen::MatrixXd global_map = gm->get_global_map();
      Eigen::MatrixXd local_map = lm->get_local_map();

      // 地図の書き出し
      gm->export_map_image(global_map);
      lm->export_map_image(local_map);
    }

    ROS_INFO("gu-ru guru");
    ros::spinOnce();
    rate.sleep();
  }
    
  delete gm;
  delete lm;

  return 0;
}
