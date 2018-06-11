#include <ros/ros.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

class GlobalMap{
public:
  GlobalMap();
  Eigen::MatrixXd get_global_map();
  Eigen::MatrixXd binarize_map(Eigen::MatrixXd img_e);
  void export_map_image(Eigen::MatrixXd img_e);
  
};

GlobalMap::GlobalMap(){};

Eigen::MatrixXd GlobalMap::get_global_map(){
  // 地図の読み込み
  cv::Mat img = cv::imread("../map/map.pgm", 0);
  if(img.empty()){
    ROS_INFO("unable to open the map");
  }else{
    ROS_INFO("map loaded");
  }
  
  // 地図データをOpenCVからEigenに渡す
  Eigen::MatrixXd img_e;
  cv::cv2eigen(img, img_e);
  ROS_INFO("opencv -> eigen");

  return img_e;
}

Eigen::MatrixXd GlobalMap::binarize_map(Eigen::MatrixXd img_e){
  // 地図を二値化する
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
  ROS_INFO("map binarized");
  return img_e;
}

void GlobalMap::export_map_image(Eigen::MatrixXd img_e){
  // 二値化された地図データをEigenからOpenCVに渡し、出力
  cv::Mat img_mono;
  cv::eigen2cv(img_e, img_mono);
  ROS_INFO("eigen -> opencv");
  cv::imwrite("../map/map_mono.pgm", img_mono);
  ROS_INFO("map exported");
}

int main(int argc, char** argv){

  GlobalMap *gm;

  gm = new GlobalMap();

  Eigen::MatrixXd img_e = gm->get_global_map();
  
  img_e = gm->binarize_map(img_e);

  gm->export_map_image(img_e);
  
  delete gm;

  return 0;
}
