#include <ros/ros.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv){
  // 地図の読み込み
  cv::Mat img = cv::imread("../map/map.pgm", 0);
  ROS_INFO("map loaded");
  
  // 地図データをOpenCVからEigenに渡す
  Eigen::MatrixXd img_e;
  cv::cv2eigen(img, img_e);

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

  // 二値化された地図データをEigenからOpenCVに渡し、出力
  cv::Mat img_mono;
  cv::eigen2cv(img_e, img_mono);
  cv::imwrite("../map/map_mono.pgm", img_mono);
  ROS_INFO("map exported");
 
  return 0;
}
