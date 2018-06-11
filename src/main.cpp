#include <ros/ros.h>
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
    ROS_INFO("global map: unable to open the map");
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
  for(int i=0; i<img_e.cols(); i++){
    for(int j=0; j<img_e.rows(); j++){
      if(img_e(i, j) > 200){
        img_e(i, j) = 255;
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
  cv::imwrite("../map/map_mono.pgm", img);
  ROS_INFO("global map: map exported");
}

int main(int argc, char** argv){

  GlobalMap *gm = new GlobalMap();

  // 地図データの読み込み
  Eigen::MatrixXd global_map = gm->get_global_map();

  gm->export_map_image(global_map);
  
  delete gm;

  return 0;
}
