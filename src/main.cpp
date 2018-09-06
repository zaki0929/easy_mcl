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
#include <cstdlib>
#include <cmath>
#include <random>
#include <thread>
#include <vector>

#define PARTICLE_NUM 100 

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

inline int check_point_within_rect(int x1, int y1, int x2, int y2, double x, double y){
  if(x >= (double)x1 && x <= (double)x2 && y >= (double)y1 && y <= (double)y2){
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
    if(global_map(-y, x) == 205){
      return 0;
    }else{
      return 1;
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

  std::uniform_int_distribution<> rand1(0, p[0].weight);    // 範囲内の一様乱数
  double M = double(weight_total) / PARTICLE_NUM;
  int r = rand1(mt);

  std::vector<int> point;
  for (int i=0; i<PARTICLE_NUM; i++){
    point.push_back(int(r+(M*i)));
  }

  Particle p_next[PARTICLE_NUM];
  int weight_sum = 0;
  int count = 0;
  int end_pick = 0;
  
  for(int i=0; i<PARTICLE_NUM; i++){
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
  for(int i=0; i<PARTICLE_NUM; i++){
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
      ROS_INFO("next");
      p[i] = p_next[i];
    }else{
      ROS_INFO("back");
      p[i] = p_temp[i];
    }
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
  //init_pose(0, 0, 0);
  odom_temp = set_pose(0, 0, 0);
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

void Particle::init_pose(int y_px, int x_px, double th){
  double resolution = 0.05;
  double x = x_px*resolution; 
  double y = -y_px*resolution; 
  ROS_INFO("y_px: %d, x_px: %d", y_px, x_px);
  ROS_INFO("y: %lf, x: %lf", y, x);

  pose = set_pose(x, y, th);
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
    rotated_data_x.push_back((data_x[i]*std::cos(pose(2))) - (data_y[i]*std::sin(pose(2))));
    rotated_data_y.push_back((data_x[i]*std::sin(pose(2))) + (data_y[i]*std::cos(pose(2))));
  }

  Eigen::MatrixXd rotated_img_e = Eigen::MatrixXd::Ones(size, size)*255; 

  for(int j=0; j<size; j++){
    for(int i=0; i<size; i++){
      int plot_toggle = 0;
      for(int k=0; k<rotated_data_x.size(); k++){
        double x_point = rotated_data_x[k] + (size/2);
        double y_point = -rotated_data_y[k] + (size/2);
        if(check_point_within_rect(i, j, i+1, j+1, x_point, y_point)){
          plot_toggle = 1;
        }
      }
      if(plot_toggle){
        rotated_img_e(j, i) = 0;
      }
    }
  }

  local_map = rotated_img_e;
}

void Particle::get_global_map(Eigen::MatrixXd img_e){
  int size_l = 240;
  int size_g = 2048;
  double resolution = 0.05;

  double x = pose(0)/resolution;
  double y = pose(1)/resolution;

  ROS_INFO("y_px: %lf, x_px: %lf", -y, x);

  Eigen::MatrixXd cut_img_e = Eigen::MatrixXd::Ones(size_l, size_l)*255; 

  for(int j=0; j<size_l; j++){
    for(int i=0; i<size_l; i++){
      if(check_point_within_rect(0, 0, size_g, size_g, x-(size_l/2)+i, -y-(size_l/2)+j)){
        if(img_e(int(-y-(size_l/2)+j), int(x-(size_l/2)+i)) == 0){
          cut_img_e(j, i) = 0;
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
  for(int j=0; j<local_map.cols(); j++){
    for(int i=0; i<local_map.rows(); i++){
      if(local_map(j, i) == 0 && global_map(j, i) == 0){
        weight++;
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

//  static tf::TransformBroadcaster br;
//  tf::Transform transform;
//  transform.setOrigin(tf::Vector3(max_weight_pose(0), max_weight_pose(1), 0.0));
//  transform.setRotation(q);
//  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "EstimatedParticle"));

}

//void Node::publish_max_weight_particle(Particle p[]){
//  geometry_msgs::Pose pose;
//
//  // 初期化
//  Eigen::Vector3d max_weight_pose = p[0].pose;
//  int max_weight = p[0].weight;
//
//  // 最も重みが大きいパーティクルの姿勢を取得
//  for(int i=0; i<PARTICLE_NUM; i++){
//    if(p[i].weight > max_weight){
//      max_weight = p[i].weight;
//      max_weight_pose(0) = p[i].pose(0);
//      max_weight_pose(1) = p[i].pose(1);
//      max_weight_pose(2) = p[i].pose(2);
//    }
//  }
//
//  // ロールピッチヨー角からクォータニオンを取得
//  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, max_weight_pose(2));
//  
//  pose.position.x = max_weight_pose(0); 
//  pose.position.y = max_weight_pose(1);
//  pose.position.z = 0;
//  
//  pose.orientation.x = q.x();
//  pose.orientation.y = q.y();
//  pose.orientation.z = q.z();
//  pose.orientation.w = q.w();
//  
//  max_wight_pose_pub.publish(pose);
//
////  static tf::TransformBroadcaster br;
////  tf::Transform transform;
////  transform.setOrigin(tf::Vector3(max_weight_pose(0), max_weight_pose(1), 0.0));
////  transform.setRotation(q);
////  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "EstimatedParticle"));
//
//}
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

  // グローバルマップを png 形式で出力
//  gm.export_map_image(global_map);

  int x1 = 973;
  int x2 = 1331;
  int y1 = 853;
  int y2 = 1161;

  // 範囲1
//  std::uniform_int_distribution<> x_px_range(700, 800);
//  std::uniform_int_distribution<> y_px_range(1750, 1850);

  // 範囲2
//  std::uniform_int_distribution<> x_px_range(614, 819);
//  std::uniform_int_distribution<> y_px_range(1536, 1741);

  // before_kidnap.pgn 用の範囲
  std::uniform_int_distribution<> x_px_range(2047-y2, 2047-y1);
  std::uniform_int_distribution<> y_px_range(x1, x2);

  std::uniform_int_distribution<> th_range(0, 360);    // 範囲内の一様乱数

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

  while(ros::ok()){

    if(n.scan_toggle){
      // scan トピックからローカルマップを生成
      lm.get_scan(n.scan);

      //p[6].get_local_map(local_map);
      //lm.export_map_image(p[6].local_map);
      
      //p[6].get_global_map(global_map);
      //lm.export_map_image(p[6].global_map);

      // ローカルマップを png 形式で出力
      //lm.export_map_image(local_map);
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

    }else{
      ROS_INFO("waiting odom topic...");
    }

    if(n.scan_toggle && n.odom_toggle){
      Eigen::MatrixXd local_map = lm.get_local_map();

      // パーティクルの重みつけを並列処理で行う
      std::vector<std::thread> threads;
      for(int i=0; i<PARTICLE_NUM; i++){
        threads.emplace_back([i, &p, &local_map, &global_map](){
          p[i].get_pose();
          p[i].get_local_map(local_map);
          p[i].get_global_map(global_map);
          p[i].calc_weight();
          ROS_INFO("%d: %d", i, p[i].weight);
        });
      }
      for(auto& t : threads){
        t.join();
      }

      //lm.export_map_image(p[0].global_map);
      //lm.export_map_image(p[0].local_map);
      resampling(p, p_temp, global_map);
      for(int i=0; i<PARTICLE_NUM; i++){
        p_temp[i] = p[i];
      }

    }

    // 最も重みの大きいパーティクルをパブリッシュ
    n.publish_max_weight_particle(p);
      
    // パーティクルをパブリッシュ
    n.publish_particle_cloud(p);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
