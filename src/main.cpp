#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdlib>
#include <cmath>

#define PARTICLE_NUM 10

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
  LocalMap();
  ~LocalMap();
  sensor_msgs::LaserScan::ConstPtr scan;
  void get_scan(sensor_msgs::LaserScan::ConstPtr _scan);
  Eigen::MatrixXd get_local_map();
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
  Eigen::Vector3d get_pose();
  Eigen::MatrixXd get_path_map();
  void init_pose(double x, double y, double th);
  void get_local_map(Eigen::MatrixXd img_e);
  void export_map_image(Eigen::MatrixXd img_e);
  double weight;
  int size_x;
  int size_y;
  Eigen::Vector3d pose;
  Eigen::Vector3d pose_base;
  Eigen::Vector3d pose_initial;
  Eigen::Vector3d odom_temp;
  Eigen::MatrixXd local_map();

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
  sensor_msgs::LaserScan::ConstPtr scan;
  nav_msgs::Odometry::ConstPtr odom;
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void publish_particle_cloud(Particle p[]);
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

inline int check_point_within_rect(int x1, int y1, int x2, int y2, double x, double y){
  if(x >= x1 && x <= x2 && y <= y1 && y >= y2){
    return 1;
  }else{
    return 0;
  }
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

LocalMap::LocalMap() : size(240){    // 240 x 240 
  homepath = std::getenv("HOME");
}

LocalMap::~LocalMap(){}

void LocalMap::get_scan(sensor_msgs::LaserScan::ConstPtr _scan){
  scan = _scan;
}

Eigen::MatrixXd LocalMap::get_local_map(){
  // 全ての要素の値が255でローカルマップサイズの行列を取得
  Eigen::MatrixXd img_e = Eigen::MatrixXd::Ones(size, size)*255; 

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

Particle::Particle(){
  // 初期化
  init_pose(0, 0, 0);
  odom_temp = set_pose(0, 0, 0);
  weight = 0;
  homepath = std::getenv("HOME");

  // 地図の読み込んでサイズを取得
//  cv::Mat img = cv::imread(homepath + "/catkin_ws/src/easy_mcl/map/map.pgm", 0);
//  if(img.empty()){
//    ROS_ERROR("particle: unable to open the map");
//  }else{
//    ROS_INFO("particle: map loaded");
//  }
//  size_x = img.rows;
//  size_y = img.cols;
//  ROS_INFO("x: %d, y: %d", size_x, size_y);

  // 軌跡をまっさらにする
//  path_map = Eigen::MatrixXd::Ones(size_y, size_x); 
//  path_map *= 255;
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

Eigen::Vector3d Particle::get_pose(){
  Eigen::Vector3d odom_pose = get_odom_pose(); 
  Eigen::Vector3d odom_diff = odom_pose - odom_temp; 
  pose_base += odom_diff;
 
  pose(0) = (pose_base(0)*std::cos(pose_initial(2))) - (pose_base(1)*std::sin(pose_initial(2)));
  pose(1) = (pose_base(0)*std::sin(pose_initial(2))) + (pose_base(1)*std::cos(pose_initial(2)));
  pose(2) = pose_base(2);

  odom_temp = odom_pose;

  return pose;
}

Eigen::MatrixXd Particle::get_path_map(){
  Eigen::Vector3d odom_pose = get_odom_pose(); 

  // 解像度に縮尺を合わせる
  int odom_x = odom_pose(0)/0.05;
  int odom_y = odom_pose(1)/0.05;

  ROS_INFO("x: %d, y: %d, th: %lf", int(odom_x), int(odom_y), odom_pose(2));
  
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

void Particle::init_pose(double x, double y, double th){
  pose = set_pose(x, y, th);
  pose_base = set_pose(x, y, th);
  pose_initial = set_pose(x, y, th);
}

void Particle::get_local_map(Eigen::MatrixXd img_e){
  int size = 240;

  std::vector<double> data_x;
  std::vector<double> data_y;

  for(int j=0; j<size; j++){
    for(int i=0; i<size; i++){
      if(img_e(j, i) == 0){
        data_x.push_back(i-(size/2)+0.5);
        data_y.push_back((size/2)-j-0.5);
      }
    }
  }

  std::vector<double> rotated_data_x;
  std::vector<double> rotated_data_y;

  for(int i=0; i<data_x.size(); i++){
    rotated_data_x.push_back((data_x*std::cos(pose(2))) - (data_y*std::sin(pose(2))));
    rotated_data_y.push_back((data_x*std::sin(pose(2))) + (data_y*std::cos(pose(2))));
  }

  Eigen::MatrixXd rotated_img_e = Eigen::MatrixXd::Ones(size, size)*255; 

  for(int j=0; j<size; j++){
    for(int i=0; i<size; i++){
      int plot_toggle = 0;
      for(int k=0; k<rotated_data_x.size(); i++){
        double x_point = rotated_data_x + (size/2) - 0.5;
        double y_point = -rotated_data_y + (size/2) - 0.5;
        if check_point_within_rect(i, j, i+1, j+1, x_point, y_point){
          plot_toggle = 1;
        }
      }
      if(plot_toggle){
        rotated_img_e(j, i);
      }
    }
  }

  local_map = rotated_img_e;
}

//----------------------------------------------------------------------

Node::Node(ros::NodeHandle& nh){
  scan_sub = nh.subscribe("scan", 1000, &Node::scanCallback, this);
  odom_sub = nh.subscribe("odom", 1000, &Node::odomCallback, this);
  pose_pub = nh.advertise<geometry_msgs::PoseArray>("ParticleCloud", 10);
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
  
  for(int i=0; i<PARTICLE_NUM; i++){
    Eigen::Vector3d pose = p[i].get_pose();

    // ロールピッチヨー角からクォータニオンを取得
    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, pose(2));

    pose_array.poses[i].position.x = pose(0); 
    pose_array.poses[i].position.y = pose(1);
    pose_array.poses[i].position.z = 0;
  
    pose_array.poses[i].orientation.x = q.x();
    pose_array.poses[i].orientation.y = q.y();
    pose_array.poses[i].orientation.z = q.z();
    pose_array.poses[i].orientation.w = q.w();
  }

  pose_pub.publish(pose_array);
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

  for(int i=0; i<PARTICLE_NUM; i++){
    p[i].init_pose(0, 0, i*0.5);
  }

//  // 地図を読み込みグローバルマップを生成
//  Eigen::MatrixXd global_map = gm.get_global_map();
//
//  // グローバルマップを png 形式で出力
//  gm.export_map_image(global_map);
//
  while(ros::ok()){
//
    if(n.scan_toggle){
      // scan トピックからローカルマップを生成
      lm.get_scan(n.scan);
      Eigen::MatrixXd local_map = lm.get_local_map();

      // ローカルマップを png 形式で出力
      lm.export_map_image(local_map);
    }else{
      ROS_INFO("waiting scan topic...");
    }

    if(n.odom_toggle){
      // odom トピックから, オドメトリをプロットした地図を生成し, png 形式で出力
//      Eigen::MatrixXd path_map = p->get_path_map();
//      p->export_map_image(path_map);

      // odom トピックから, パーティクルの位置姿勢を計算しパブリッシュ
      for(int i=0; i<PARTICLE_NUM; i++){
        p[i].get_odom(n.odom);
      }
      n.publish_particle_cloud(p);

    }else{
      ROS_INFO("waiting odom topic...");
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
