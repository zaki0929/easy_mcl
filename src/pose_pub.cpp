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

class Pose{
public:
  Pose(ros::NodeHandle& nh);
  ~Pose();

}


class Node{
public:
  Node(ros::NodeHandle& nh);
  ~Node();
  ros::Subscriber odom_sub;
  ros::Subscriber pose_array_sub
  ros::Publisher pose_array_pub;
  geometry_msgs::PoseArray::ConstPtr pose_array;
  nav_msgs::Odometry::ConstPtr odom;
  void poseCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void publish_pose_array(Particle p[]);
  int pose_array_toggle;
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

//----------------------------------------------------------------------

Node::Node(ros::NodeHandle& nh){
  odom_sub = nh.subscribe("odom", 1000, &Node::odomCallback, this);
  pose_array_sub = nh.subscribe("EstimatedParticle", 1000, &Node::poseCallback, this);
  pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("pose", 10);
  odom_toggle = 0;
  pose_array_toggle = 0;
}

Node::~Node(){}

void Node::poseCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
  if(!pose_array_toggle){
    pose_array_toggle = 1;
  }
  pose_array = msg;
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
  Particle p[PARTICLE_NUM];
  Particle p_temp[PARTICLE_NUM];

  while(ros::ok()){

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

      // 最も重みの大きいパーティクルをパブリッシュ
      n.publish_max_weight_particle(p);
      
      // パーティクルをパブリッシュ
      n.publish_particle_cloud(p);

    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
