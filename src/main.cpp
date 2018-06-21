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
  Particle(ros::NodeHandle& nh, double x, double y);
  ~Particle();
  ros::Subscriber odom_sub;
  ros::Publisher pose_pub;
  nav_msgs::Odometry::ConstPtr odom;
  Eigen::Vector3d get_odom_pose();
  Eigen::Vector3d get_pose();
  Eigen::MatrixXd get_path_map();
  void scanCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void publish_particle_cloud(Eigen::Vector3d pose);
  void export_map_image(Eigen::MatrixXd img_e);
  double weight;
  int odom_toggle;
  int size_x;
  int size_y;
  Eigen::Vector3d pose;
  Eigen::Vector3d pose_diff;
  Eigen::Vector3d pose_temp;

private:
  std::string homepath;
  Eigen::MatrixXd path_map;
};

//----------------------------------------------------------------------

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

Particle::Particle(ros::NodeHandle& nh, double x, double y){
  // 初期化
  pose = set_pose(x, y, 0);
  pose_temp = set_pose(x, y, 0);
  weight = 0;
  odom_toggle = 0;
  homepath = std::getenv("HOME");
  odom_sub = nh.subscribe("odom", 1000, &Particle::scanCallback, this);
  pose_pub = nh.advertise<geometry_msgs::PoseArray>("ParticleCloud", 10);

  // 地図の読み込んでサイズを取得
  cv::Mat img = cv::imread(homepath + "/catkin_ws/src/easy_mcl/map/map.pgm", 0);
  if(img.empty()){
    ROS_ERROR("particle: unable to open the map");
  }else{
    ROS_INFO("particle: map loaded");
  }
  size_x = img.rows;
  size_y = img.cols;
  ROS_INFO("x: %d, y: %d", size_x, size_y);



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

Eigen::Vector3d Particle::get_odom_pose(){
  tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);

  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  Eigen::Vector3d odom_pose = set_pose(odom->pose.pose.position.x, odom->pose.pose.position.y, yaw); 
  return odom_pose;
} 

Eigen::Vector3d Particle::get_pose(){
  Eigen::Vector3d odom_pose = get_odom_pose(); 

  pose_diff = odom_pose - pose_temp;
  pose += pose_diff;

  pose_temp = pose;

  return pose;
}

void Particle::publish_particle_cloud(Eigen::Vector3d pose){
  geometry_msgs::PoseArray pose_array;

  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, pose(2));

  pose_array.header.stamp = ros::Time::now();
  pose_array.header.frame_id = "map";
  pose_array.poses.resize(1);            // パーティクル数

  pose_array.poses[0].position.x = pose(0); 
  pose_array.poses[0].position.y = pose(1);
  pose_array.poses[0].position.z = 0;

  pose_array.poses[0].orientation.x = q.x();
  pose_array.poses[0].orientation.y = q.y();
  pose_array.poses[0].orientation.z = q.z();
  pose_array.poses[0].orientation.w = q.w();

  pose_pub.publish(pose_array);
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

//----------------------------------------------------------------------

int main(int argc, char** argv){
  ros::init(argc, argv, "easy_mcl");
  ros::NodeHandle nh;
  ros::Rate rate(10);

  GlobalMap *gm = new GlobalMap();
  LocalMap *lm = new LocalMap(nh);
  Particle *p = new Particle(nh, 0, 0);

  // 地図を読み込みグローバルマップを生成
  Eigen::MatrixXd global_map = gm->get_global_map();

  // グローバルマップを png 形式で出力
  gm->export_map_image(global_map);

  while(ros::ok()){

    if(lm->scan_toggle){
      // scan トピックからローカルマップを生成
      Eigen::MatrixXd local_map = lm->get_local_map();

      // ローカルマップを png 形式で出力
      lm->export_map_image(local_map);
    }else{
      ROS_INFO("waiting scan topic...");
    }

    if(p->odom_toggle){
      // odom トピックから, オドメトリをプロットした地図を生成しi, png 形式で出力
//      Eigen::MatrixXd path_map = p->get_path_map();
//      p->export_map_image(path_map);

      // odom トピックから, パーティクルの位置姿勢を計算しパブリッシュ
      p->publish_particle_cloud(p->get_pose());
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
