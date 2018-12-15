#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cstdlib>
#include <cmath>
#include <random>
#include <thread>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>

#define BLUR_LOCAL_MAP 1                      // 1: true, 0: false

#define PRECASTING 1                          // 1: true, 0: false
#define SCAN_RANGE_MAX 5.6

#define USE_WHITE_GRID_TO_CALCULATE_WEIGHT 0  // 1: true, 0: false

#define PARTICLE_NUM 300
#define RATE_OF_RANDOM_PARTICLE 0.1

#define RANGE_X1 970
#define RANGE_Y1 850
#define RANGE_X2 1330
#define RANGE_Y2 1250

class GlobalMap{
public:
  GlobalMap();
  ~GlobalMap();
  Eigen::MatrixXd get_global_map();
  Eigen::MatrixXd binarize_map(Eigen::MatrixXd img_e);
  Eigen::MatrixXd blur_map(Eigen::MatrixXd img_e, int target, int val);
  void export_map_image(Eigen::MatrixXd img_e);

private:
  std::string homepath;
};

class LocalMap{
public:
  LocalMap();
  ~LocalMap();
  sensor_msgs::LaserScan::ConstPtr scan;
  void get_scan(sensor_msgs::LaserScan::ConstPtr _scan);
  Eigen::MatrixXd get_local_map();
  Eigen::MatrixXd blur_map(Eigen::MatrixXd img_e, int target, int val);
  void export_map_image(Eigen::MatrixXd img_e);
  const int size;

private:
  std::string homepath;
};

class Particle{
public:
  Particle(ros::NodeHandle& nh, double x, double y, double th);
  Particle();
  ~Particle();
  nav_msgs::Odometry::ConstPtr odom;
  void get_odom(nav_msgs::Odometry::ConstPtr _odom);
  Eigen::Vector3d get_odom_pose();
  Eigen::MatrixXd get_path_map();
  void get_pose();
  void init_pose(int y_px, int x_px, double th);
  void get_local_map(Eigen::MatrixXd img_e);
  void get_global_map(Eigen::MatrixXd img_e);
  void calc_weight();
  void export_map_image(Eigen::MatrixXd img_e);
  int weight;
  int size_x;
  int size_y;
  Eigen::Vector3d pose;
  Eigen::Vector3d pose_initial;
  Eigen::Vector3d odom_temp;
  Eigen::MatrixXd local_map;
  Eigen::MatrixXd global_map;

private:
  std::string homepath;
  Eigen::MatrixXd path_map;
};

class Node{
public:
  Node(ros::NodeHandle& nh);
  ~Node();
  ros::Subscriber scan_sub;
  ros::Subscriber odom_sub;
  ros::Publisher pose_pub;
  ros::Publisher max_wight_pose_pub;
  sensor_msgs::LaserScan::ConstPtr scan;
  nav_msgs::Odometry::ConstPtr odom;
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void publish_particle_cloud(Particle p[]);
  void publish_max_weight_particle(Particle p[]);
  int scan_toggle;
  int odom_toggle;
};

//----------------------------------------------------------------------

inline Eigen::Vector3d set_pose(double x, double y, double th){
  Eigen::Vector3d vec;
  vec(0) = x;
  vec(1) = y;
  vec(2) = th;
  return vec;
}

// 2本の線分が交わるかどうかを判定する関数
inline int check_intersection(int ax, int ay, int bx, int by, int cx, int cy, int dx, int dy){
  double ta = (cx - dx) * (ay - cy) + (cy - dy) * (cx - ax);
  double tb = (cx - dx) * (by - cy) + (cy - dy) * (cx - bx);
  double tc = (ax - bx) * (cy - ay) + (ay - by) * (ax - cx);
  double td = (ax - bx) * (dy - ay) + (ay - by) * (ax - dx);

  if(tc * td <= 0 && ta * tb <= 0){
    if((ay - by) * (cx - dx) == (cy - dy) * (ax - bx)){
      return 0;
    }else{
      return 1;
    }
  }else{
    return 0;
  }
}

// 矩形と線分が交わるかどうかを判定する関数
inline int check_intersection_rect_line(int x1, int y1, int x2, int y2, int ax, int ay, int bx, int by){
  if(check_intersection(x1, y1, x1, y2, ax, ay, bx, by) || check_intersection(x1, y2, x2, y2, ax, ay, bx, by) || check_intersection(x2, y2, x2, y1, ax, ay, bx, by) || check_intersection(x2, y1, x1, y1, ax, ay, bx, by)){
    return 1;
  }else{
    return 0;
  }
}


// 点の座標が矩形の中にあるか判断する関数
inline int check_point_within_rect(int x1, int y1, int x2, int y2, double x, double y){
  if(x >= (double)x1 && x <= (double)x2 && y >= (double)y1 && y <= (double)y2){
    return 1;
  }else{
    return 0;
  }
}

// 点の座標が円の中にあるか判断する関数
inline int check_point_within_circle(int x, int y, double r){
  if(pow(x, 2) + pow(y, 2) < pow(r, 2)){
    return 1;
  }else{
    return 0;
  }
}

// パーティクルが地図上にあるか判断する関数
inline int is_on_global_map(Particle p, Eigen::MatrixXd global_map, double resolution){
  double x = p.pose(0)/resolution;
  double y = p.pose(1)/resolution;

  if(check_point_within_rect(0, 0, 2047, 2047, x, -y)){
    if(global_map(-y, x) != 205 && global_map(-y, x) != 0){
      return 1;
    }else{
      return 0;
    }
  }else{
    return 0;
  }
}


inline void resampling(Particle p[], Particle p_temp[], Eigen::MatrixXd global_map){
  
  // 等間隔リサンプリング
  std::random_device rnd;
  std::mt19937 mt(rnd());

  int weight_total = 0;
  for(int i=0; i<PARTICLE_NUM; i++){
    weight_total += p[i].weight;
  }

  int random_particle_num;
  int resampling_particle_num;
  if(PARTICLE_NUM >= 10){
    random_particle_num = int(PARTICLE_NUM * RATE_OF_RANDOM_PARTICLE);
    resampling_particle_num = PARTICLE_NUM - random_particle_num;
  }

  std::uniform_int_distribution<> rand1(0, p[0].weight);    // 範囲内の一様乱数
  double M = double(weight_total / resampling_particle_num);
  int r = rand1(mt);

  std::vector<int> point;
  for (int i=0; i<resampling_particle_num; i++){
    point.push_back(int(r+(M*i)));
  }

  Particle p_next[PARTICLE_NUM];
  int weight_sum = 0;
  int count = 0;
  int end_pick = 0;
  
  for(int i=0; i<resampling_particle_num; i++){
    int weight_sum_temp = weight_sum; 
    weight_sum += p[i].weight;
    while(point[count] >= weight_sum_temp && point[count] < weight_sum && end_pick == 0){
      p_next[count].pose = p[i].pose;
      p_next[count].pose_initial = p[i].pose_initial;
      p_next[count].odom_temp = p[i].odom_temp;
      if(count < point.size()-1){
        count += 1;
      }else{
        end_pick = 1;
      }
    }
  }

  // パーティクルの位置を少しずらす
  std::normal_distribution<> norm1(0, 0.2);    // 正規分布: 平均0, 分散0.2
  std::normal_distribution<> norm2(0, 0.01);    // 正規分布: 平均0, 分散0.01
  for(int i=0; i<resampling_particle_num; i++){
    Eigen::Vector3d nomal_error;
    nomal_error(0) = norm1(mt);
    nomal_error(1) = norm1(mt);
    nomal_error(2) = norm2(mt); 

    //ROS_INFO("-> 1: %lf, 2: %lf, 3: %lf", nomal_error(0), nomal_error(1), nomal_error(2));

    p_next[i].pose += nomal_error;
    p_next[i].pose_initial += nomal_error;
    p_next[i].odom_temp += nomal_error;

    // グレーゾーンにパーティクルがいってしまった場合引き戻す
    if(is_on_global_map(p_next[i], global_map, 0.05)){
      p[i] = p_next[i];
    }else{
      p[i] = p_temp[i];
    }
  }

  // ランダムパーティクルの注入
  std::uniform_int_distribution<> x_px_range(2047-RANGE_Y2, 2047-RANGE_Y1);
  std::uniform_int_distribution<> y_px_range(RANGE_X1, RANGE_X2);
  std::uniform_int_distribution<> th_range(0, 360);

  for(int i=0; i<random_particle_num; i++){
    int is_on_global_map_toggle = 0;
    while(!is_on_global_map_toggle){
      p[resampling_particle_num + i].init_pose(x_px_range(mt), y_px_range(mt), double(th_range(mt))*3.14/180);
      if(is_on_global_map(p[resampling_particle_num + i], global_map, 0.05)){
        is_on_global_map_toggle = 1;
      }
    }
  }
}

inline Particle debug_max_weight_particle(Particle p[]){
  Particle max_weight_particle = p[0];
  int max_weight = p[0].weight;

  // 最も重みが大きいパーティクルをさがす
  for(int i=0; i<PARTICLE_NUM; i++){
    if(p[i].weight > max_weight){
      max_weight_particle = p[i];
      max_weight = p[i].weight;
    }
  }
  return max_weight_particle;
}

struct record{
  int row;
  int col;
  double distance;
};

inline bool cust_predicate(record elem1, record elem2){
  if(elem1.distance < elem2.distance){
    return true;
  }else{
    return false;
  }
}

inline void precasting(double scan_angle_min, double scan_angle_max, double scan_angle_increment){
  ROS_INFO("precasting: start");
  const int size = 240;
  int scan_index=0;
  for(double th=scan_angle_min; th<=scan_angle_max; th+=scan_angle_increment){
    int x = int(SCAN_RANGE_MAX*std::cos(th)/0.05);
    int y = int(SCAN_RANGE_MAX*std::sin(th)/0.05);

    std::string homepath = std::getenv("HOME");
    std::string file_location = homepath + "/catkin_ws/src/easy_mcl/resources/precasting/";
    std::string file_name = std::to_string(scan_index);
    std::string file_format = ".csv";
    std::string file_info = file_location + file_name + file_format;

    std::ofstream ofs(file_info.c_str());
    if(!ofs){
      ROS_INFO("precasting: failed");
      return;
    }

    std::vector<record> records;
    for(int k=0; k<size; k++){
      for(int j=0; j<size; j++){
        if(check_intersection_rect_line(-int(size/2)+j, int(size/2)-k, -int(size/2)+j+1, int(size/2)-k-1, 0, 0, x, y)){
          record r;
          r.row = k;
          r.col = j;
          r.distance = sqrt(pow(-int(size/2)+j, 2) + pow(int(size/2)-k, 2));
          records.push_back(r);
        }
      }
    }
    
    // 距離でソート 
    stable_sort(records.begin(), records.end(), &cust_predicate);

    for(record r : records){
      std::string record_str = std::to_string(r.row) + "," + std::to_string(r.col) + "," + std::to_string(r.distance);
      ofs << record_str.c_str() << std::endl;
    }
   
    scan_index++;
  }
  ROS_INFO("precasting: finish");
}

inline Eigen::MatrixXd raycasting(int scan_index, double range, Eigen::MatrixXd img_e, int size){
  if(range > 0){
    double distance = range/0.05;

    std::string homepath = std::getenv("HOME");
    std::string file_location = homepath + "/catkin_ws/src/easy_mcl/resources/precasting/";
    std::string file_name = std::to_string(scan_index);
    std::string file_format = ".csv";
    std::string file_info = file_location + file_name + file_format;

    std::ifstream ifs(file_info.c_str());
    if(!ifs){
      ROS_INFO("raycasting: failed to open the csv");
    }

    std::string line; 
    while(std::getline(ifs, line)){
      int index = 0;
      std::stringstream ss(line);
      std::string data;
      record r;
      while(std::getline(ss, data, ',')){
        switch(index){
          case 0: r.row = std::stoi(data); break;
          case 1: r.col = std::stoi(data); break;
          case 2: r.distance = std::stod(data); break;
        }

        if(index == 2){
          if(r.distance < distance){
            img_e(r.row, r.col) = 255;
          }else{
            return img_e;
          }  
        }

        index++;
      }
    }
  }
  return img_e;
}

//----------------------------------------------------------------------

GlobalMap::GlobalMap(){
  homepath = std::getenv("HOME");
}

GlobalMap::~GlobalMap(){}

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

  // 地図をぼかす
  img_e = blur_map(img_e, 0, 20);
  img_e = blur_map(img_e, 20, 40);
  img_e = blur_map(img_e, 40, 60);
  img_e = blur_map(img_e, 60, 80);
  img_e = blur_map(img_e, 80, 100);
  img_e = blur_map(img_e, 100, 120);
  img_e = blur_map(img_e, 120, 140);
  img_e = blur_map(img_e, 140, 160);
  img_e = blur_map(img_e, 160, 180);
  img_e = blur_map(img_e, 180, 200);

  return img_e;
}

Eigen::MatrixXd GlobalMap::binarize_map(Eigen::MatrixXd img_e){
  ROS_INFO("global map: %lf", img_e(0, 0));
  // 200を閾値にして地図を二値化する
  for(int i=0; i<img_e.rows(); i++){
    for(int j=0; j<img_e.cols(); j++){
      if(img_e(j, i) > 205){
        img_e(j, i) = 255;
      }
      else if(img_e(j, i) < 205){
        img_e(j, i) = 0;
      }
    }
  }
  ROS_INFO("global map: binarized");
  return img_e;
}

Eigen::MatrixXd GlobalMap::blur_map(Eigen::MatrixXd img_e, int target, int val){
  for(int i=1; i<img_e.rows()-1; i++){
    for(int j=1; j<img_e.cols()-1; j++){
      if(img_e(j, i) == target){
        if(img_e(j+1, i-1) > target){
          img_e(j+1, i-1) = val;
        }
        if(img_e(j+1, i) > target){
          img_e(j+1, i) = val;
        }
        if(img_e(j+1, i+1) > target){
          img_e(j+1, i+1) = val;
        }
        if(img_e(j, i-1) > target){
          img_e(j, i-1) = val;
        }
        if(img_e(j, i+1) > target){
          img_e(j, i+1) = val;
        }
        if(img_e(j-1, i-1) > target){
          img_e(j-1, i-1) = val;
        }
        if(img_e(j-1, i) > target){
          img_e(j-1, i) = val;
        }
        if(img_e(j-1, i+1) > target){
          img_e(j-1, i+1) = val;
        }
      }
    }
  }
  ROS_INFO("global map: blur");
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

LocalMap::LocalMap() : size(240){    // 240 x 240 
  homepath = std::getenv("HOME");
}

LocalMap::~LocalMap(){}

void LocalMap::get_scan(sensor_msgs::LaserScan::ConstPtr _scan){
  scan = _scan;
}

Eigen::MatrixXd LocalMap::get_local_map(){
  ROS_INFO("raycasting: start");

  // 全ての要素の値が205でローカルマップサイズの行列を取得
  Eigen::MatrixXd img_e = Eigen::MatrixXd::Ones(size, size)*205; 

  int scan_index = 0;
  // レイキャスティング
  scan_index = 0;
  for(double th=scan->angle_min; th<=scan->angle_max; th+=scan->angle_increment){
    img_e = raycasting(scan_index, scan->ranges[scan_index], img_e, size);
    scan_index++;
  }
  
  // センサ値のプロット
  scan_index = 0;
  for(double th=scan->angle_min; th<=scan->angle_max; th+=scan->angle_increment){
    if(scan->ranges[scan_index] > 0){
      int x = int(scan->ranges[scan_index]*std::cos(th)/0.05);
      int y = int(scan->ranges[scan_index]*std::sin(th)/0.05);
      img_e(int(size/2)-y, int(size/2)+x) = 0;
    }
    scan_index++;
  }
  
  if(BLUR_LOCAL_MAP){
    img_e = blur_map(img_e, 0, 0);
  }

  ROS_INFO("raycasting: finish");
  ROS_INFO("local map: completed writing map");
  return img_e;
}

Eigen::MatrixXd LocalMap::blur_map(Eigen::MatrixXd img_e, int target, int val){
  Eigen::MatrixXd blur_img_e = img_e;
  for(int i=1; i<img_e.rows()-1; i++){
    for(int j=1; j<img_e.cols()-1; j++){
      if(img_e(j, i) == target){
        blur_img_e(j+1, i-1) = val;
        blur_img_e(j+1, i) = val;
        blur_img_e(j+1, i+1) = val;
        blur_img_e(j, i-1) = val;
        blur_img_e(j, i+1) = val;
        blur_img_e(j-1, i-1) = val;
        blur_img_e(j-1, i) = val;
        blur_img_e(j-1, i+1) = val;
      }
    }
  }
  ROS_INFO("local map: blur");
  return blur_img_e;
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

Particle::Particle(){
  // 初期化
  //init_pose(0, 0, 0);
  odom_temp = set_pose(0, 0, 0);
  homepath = std::getenv("HOME");

  //// 地図の読み込んでサイズを取得
  //cv::Mat img = cv::imread(homepath + "/catkin_ws/src/easy_mcl/map/map.pgm", 0);
  //if(img.empty()){
  //  ROS_ERROR("particle: unable to open the map");
  //}else{
  //  ROS_INFO("particle: map loaded");
  //}
  //size_x = img.rows;
  //size_y = img.cols;
  //ROS_INFO("x: %d, y: %d", size_x, size_y);

  //// 軌跡をまっさらにする
  //path_map = Eigen::MatrixXd::Ones(size_y, size_x); 
  //path_map *= 255;
}

Particle::~Particle(){}

void Particle::get_odom(nav_msgs::Odometry::ConstPtr _odom){
  odom = _odom;
}

Eigen::Vector3d Particle::get_odom_pose(){
  tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);

  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  Eigen::Vector3d odom_pose = set_pose(odom->pose.pose.position.x, odom->pose.pose.position.y, yaw); 

  return odom_pose;
} 

void Particle::get_pose(){
  Eigen::Vector3d odom_pose = get_odom_pose(); 
  Eigen::Vector3d odom_diff = odom_pose - odom_temp; 
 
  pose(0) += (odom_diff(0)*std::cos(pose_initial(2))) - (odom_diff(1)*std::sin(pose_initial(2)));
  pose(1) += (odom_diff(0)*std::sin(pose_initial(2))) + (odom_diff(1)*std::cos(pose_initial(2)));
  pose(2) += odom_diff(2);

  odom_temp = odom_pose;
}

Eigen::MatrixXd Particle::get_path_map(){
  Eigen::Vector3d odom_pose = get_odom_pose(); 

  // 解像度に縮尺を合わせる
  int odom_x = odom_pose(0)/0.05;
  int odom_y = odom_pose(1)/0.05;

  //ROS_INFO("x: %d, y: %d, th: %lf", int(odom_x), int(odom_y), odom_pose(2));
  
  // 位置を描画
  path_map(int(size_y/2)-odom_y, int(size_x/2)+odom_x) = 0;

  // 向きを描画
  for(int i=0; i<5; i++){
    path_map(int(size_y/2)-odom_y-int(std::cos(odom_pose(2))*2*(i+1)), int(size_x/2)+odom_x+int(std::sin(odom_pose(2))*2*(i+1))) = 240-(20*i);
  }

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

void Particle::init_pose(int y_px, int x_px, double th){
  double resolution = 0.05;
  double x = x_px*resolution; 
  double y = -y_px*resolution; 
  //ROS_INFO("y_px: %d, x_px: %d", y_px, x_px);
  //ROS_INFO("y: %lf, x: %lf", y, x);

  pose = set_pose(x, y, th);
  pose_initial = set_pose(x, y, th);
}

void Particle::get_local_map(Eigen::MatrixXd img_e){
  cv::Mat img;
  cv::Mat rotated_img;
  cv::eigen2cv(img_e, img);

  float angle = pose(2)*180/M_PI;
  float scale = 1.0;
  cv::Point2f center(img.cols/2.0, img.rows/2.0);

  // アフィン変換行列の取得
  cv::Mat affine = cv::getRotationMatrix2D(center, angle, scale);
 
  warpAffine(img, rotated_img, affine, img.size());

  cv::cv2eigen(rotated_img, img_e);

  local_map = img_e;
}

void Particle::get_global_map(Eigen::MatrixXd img_e){
  int size_l = 240;
  int size_g = 2048;
  double resolution = 0.05;

  double x = pose(0)/resolution;
  double y = pose(1)/resolution;

  //ROS_INFO("y_px: %lf, x_px: %lf", -y, x);

  Eigen::MatrixXd cut_img_e = Eigen::MatrixXd::Ones(size_l, size_l)*255; 

  for(int j=0; j<size_l; j++){
    for(int i=0; i<size_l; i++){
      if(check_point_within_rect(0, 0, size_g, size_g, x-(size_l/2)+i, -y-(size_l/2)+j)){
        if(img_e(int(-y-(size_l/2)+j), int(x-(size_l/2)+i)) == 0){
          cut_img_e(j, i) = 0;
        }
        if(img_e(int(-y-(size_l/2)+j), int(x-(size_l/2)+i)) == 20){
          cut_img_e(j, i) = 20;
        }
        if(img_e(int(-y-(size_l/2)+j), int(x-(size_l/2)+i)) == 40){
          cut_img_e(j, i) = 40;
        }
        if(img_e(int(-y-(size_l/2)+j), int(x-(size_l/2)+i)) == 60){
          cut_img_e(j, i) = 60;
        }
        if(img_e(int(-y-(size_l/2)+j), int(x-(size_l/2)+i)) == 80){
          cut_img_e(j, i) = 80;
        }
        if(img_e(int(-y-(size_l/2)+j), int(x-(size_l/2)+i)) == 100){
          cut_img_e(j, i) = 100;
        }
        if(img_e(int(-y-(size_l/2)+j), int(x-(size_l/2)+i)) == 120){
          cut_img_e(j, i) = 120;
        }
        if(img_e(int(-y-(size_l/2)+j), int(x-(size_l/2)+i)) == 140){
          cut_img_e(j, i) = 140;
        }
        if(img_e(int(-y-(size_l/2)+j), int(x-(size_l/2)+i)) == 160){
          cut_img_e(j, i) = 160;
        }
        if(img_e(int(-y-(size_l/2)+j), int(x-(size_l/2)+i)) == 180){
          cut_img_e(j, i) = 180;
        }
        if(img_e(int(-y-(size_l/2)+j), int(x-(size_l/2)+i)) == 200){
          cut_img_e(j, i) = 200;
        }
        if(img_e(int(-y-(size_l/2)+j), int(x-(size_l/2)+i)) == 205){
          cut_img_e(j, i) = 205;
        }
      }
    }
  }
  global_map = cut_img_e;
}

void Particle::calc_weight(){
  weight = 0;
  int size = local_map.cols();
  for(int j=0; j<local_map.cols(); j++){
    for(int i=0; i<local_map.rows(); i++){
      if(check_point_within_circle(-int(size/2)+i, int(size/2)-j, int(size/2))){ 
        if(local_map(j, i) == 0 && global_map(j, i) == 0){
          weight += 1000;
        }
        if(local_map(j, i) == 0 && global_map(j, i) == 20){
          weight += 900;
        }
        if(local_map(j, i) == 0 && global_map(j, i) == 40){
          weight += 800;
        }
        if(local_map(j, i) == 0 && global_map(j, i) == 60){
          weight += 700;
        }
        if(local_map(j, i) == 0 && global_map(j, i) == 80){
          weight += 600;
        }
        if(local_map(j, i) == 0 && global_map(j, i) == 100){
          weight += 500;
        }
        if(local_map(j, i) == 0 && global_map(j, i) == 120){
          weight += 400;
        }
        if(local_map(j, i) == 0 && global_map(j, i) == 140){
          weight += 300;
        }
        if(local_map(j, i) == 0 && global_map(j, i) == 160){
          weight += 200;
        }
        if(local_map(j, i) == 0 && global_map(j, i) == 180){
          weight += 100;
        }
        if(local_map(j, i) == 0 && global_map(j, i) == 200){
          weight += 50;
        }
        if(local_map(j, i) == 0 && (global_map(j, i) == 205 || global_map(j, i) == 255)){
	  if(weight > 100){
            weight -= 1000;
	  }else{
	    weight = 0;
	  }
	}
	if(USE_WHITE_GRID_TO_CALCULATE_WEIGHT){
          if(local_map(j, i) == 255 && (global_map(j, i) == 255)){
            weight += 10;
          }
          if(local_map(j, i) == 255 && global_map(j, i) == 205){
	    if(weight > 1){
              weight -= 10;
	    }
          }
	}
      }
    }
  }
}

//----------------------------------------------------------------------

Node::Node(ros::NodeHandle& nh){
  scan_sub = nh.subscribe("scan", 1000, &Node::scanCallback, this);
  odom_sub = nh.subscribe("odom", 1000, &Node::odomCallback, this);
  pose_pub = nh.advertise<geometry_msgs::PoseArray>("ParticleCloud", 10);
  max_wight_pose_pub = nh.advertise<geometry_msgs::PoseArray>("EstimatedParticle", 10);
  scan_toggle = 0;
  odom_toggle = 0;
}

Node::~Node(){}

void Node::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(scan_toggle){
    //ROS_INFO("local map: get data from scan topic");
  }else{
    scan_toggle = 1;
  }
  scan = msg;
}

void Node::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  if(odom_toggle){
    //ROS_INFO("particle: get data from odom topic");
  }else{
    odom_toggle = 1;
  }
  odom = msg;
}

void Node::publish_particle_cloud(Particle p[]){
  geometry_msgs::PoseArray pose_array;
  
  pose_array.header.stamp = ros::Time::now();
  pose_array.header.frame_id = "map";
  pose_array.poses.resize(PARTICLE_NUM);
  
  std::vector<std::thread> threads;
  for(int i=0; i<PARTICLE_NUM; i++){
    threads.emplace_back([i, &p, &pose_array](){
      // ロールピッチヨー角からクォータニオンを取得
      tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, p[i].pose(2));
  
      pose_array.poses[i].position.x = p[i].pose(0); 
      pose_array.poses[i].position.y = p[i].pose(1);
      pose_array.poses[i].position.z = 0;
    
      pose_array.poses[i].orientation.x = q.x();
      pose_array.poses[i].orientation.y = q.y();
      pose_array.poses[i].orientation.z = q.z();
      pose_array.poses[i].orientation.w = q.w();
    });
  }
  for(auto& t : threads){
    t.join();
  }

  pose_pub.publish(pose_array);
}

void Node::publish_max_weight_particle(Particle p[]){
  geometry_msgs::PoseArray pose_array;

  // 初期化
  pose_array.header.stamp = ros::Time::now();
  pose_array.header.frame_id = "map";
  pose_array.poses.resize(1);
  Eigen::Vector3d max_weight_pose = p[0].pose;
  int max_weight = p[0].weight;

  // 最も重みが大きいパーティクルの姿勢を取得
  for(int i=0; i<PARTICLE_NUM; i++){
    if(p[i].weight > max_weight){
      max_weight = p[i].weight;
      max_weight_pose(0) = p[i].pose(0);
      max_weight_pose(1) = p[i].pose(1);
      max_weight_pose(2) = p[i].pose(2);
    }
  }

  // ロールピッチヨー角からクォータニオンを取得
  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, max_weight_pose(2));
  
  pose_array.poses[0].position.x = max_weight_pose(0); 
  pose_array.poses[0].position.y = max_weight_pose(1);
  pose_array.poses[0].position.z = 0;
  
  pose_array.poses[0].orientation.x = q.x();
  pose_array.poses[0].orientation.y = q.y();
  pose_array.poses[0].orientation.z = q.z();
  pose_array.poses[0].orientation.w = q.w();
  
  max_wight_pose_pub.publish(pose_array);
}

//----------------------------------------------------------------------

int main(int argc, char** argv){
  ros::init(argc, argv, "easy_mcl");
  ros::NodeHandle nh;
  ros::Rate rate(0.5);

  Node n(nh);
  GlobalMap gm;
  LocalMap lm;
  Particle p[PARTICLE_NUM];
  Particle p_temp[PARTICLE_NUM];

  std::random_device rnd;
  std::mt19937 mt(rnd());

  // 地図を読み込みグローバルマップを生成
  Eigen::MatrixXd global_map = gm.get_global_map();

  //// グローバルマップを png 形式で出力
  //gm.export_map_image(global_map);

  std::uniform_int_distribution<> x_px_range(2047-RANGE_Y2, 2047-RANGE_Y1);
  std::uniform_int_distribution<> y_px_range(RANGE_X1, RANGE_X2);
  std::uniform_int_distribution<> th_range(0, 360);

  // パーティクル位置の初期化
  for(int i=0; i<PARTICLE_NUM; i++){
    int is_on_global_map_toggle = 0;
    while(!is_on_global_map_toggle){
      p[i].init_pose(x_px_range(mt), y_px_range(mt), double(th_range(mt))*3.14/180);
      if(is_on_global_map(p[i], global_map, 0.05)){
        is_on_global_map_toggle = 1;
      }
    }
    p_temp[i] = p[i];
  }

  int precasting_toggle = 1;
  while(ros::ok()){

    if(n.scan_toggle){
      // scan トピックからローカルマップを生成
      lm.get_scan(n.scan);

      if(PRECASTING && precasting_toggle){
        precasting(lm.scan->angle_min, lm.scan->angle_max, lm.scan->angle_increment);
	precasting_toggle = 0;
      }

      //// ローカルマップを png 形式で出力
      //lm.export_map_image(local_map);
    }else{
      ROS_INFO("waiting scan topic...");
    }

    if(n.odom_toggle){
      //// odom トピックから, オドメトリをプロットした地図を生成し, png 形式で出力
      //Eigen::MatrixXd path_map = p->get_path_map();
      //p->export_map_image(path_map);

      // odom トピックから, パーティクルの位置姿勢を計算しパブリッシュ
      for(int i=0; i<PARTICLE_NUM; i++){
        p[i].get_odom(n.odom);
      }

    }else{
      ROS_INFO("waiting odom topic...");
    }

    if(n.scan_toggle && n.odom_toggle){
      Eigen::MatrixXd local_map = lm.get_local_map();

      // パーティクルの重みつけを並列処理で行う
      ROS_INFO("particle: calculating weight...");
      std::vector<std::thread> threads;
      for(int i=0; i<PARTICLE_NUM; i++){
        threads.emplace_back([i, &p, &local_map, &global_map](){
          p[i].get_pose();
          p[i].get_local_map(local_map);
          p[i].get_global_map(global_map);
          p[i].calc_weight();
          //ROS_INFO("%d: %d", i, p[i].weight);
        });
      }
      for(auto& t : threads){
        t.join();
      }

      // Debug
      Particle max_weight_particle = debug_max_weight_particle(p);
      gm.export_map_image(max_weight_particle.global_map);
      lm.export_map_image(max_weight_particle.local_map);

      // 最も重みの大きいパーティクルをパブリッシュ
      n.publish_max_weight_particle(p);

      ROS_INFO("particle: resamping...");
      resampling(p, p_temp, global_map);
      for(int i=0; i<PARTICLE_NUM; i++){
        p_temp[i] = p[i];
      }
    }
      
    // パーティクルをパブリッシュ
    n.publish_particle_cloud(p);
    ROS_INFO("particle: published");

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
