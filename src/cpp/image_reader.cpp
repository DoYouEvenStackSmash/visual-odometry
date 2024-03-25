#include <opencv2/opencv.hpp>
#include <iostream>
#include <eigen3/Eigen/Core>
#include "kernels.h"
void preprocess_image(cv::Mat &b) {
  cv::cvtColor(b.t(), b, cv::COLOR_BGR2GRAY);
  cv::flip(b, b, 1);
}

Eigen::MatrixXd mat2eig(const cv::Mat &mat) {
  if (mat.empty()){
    std::cerr << "input matrix is empty" << std::endl;
    return Eigen::MatrixXd();
  }
  Eigen::MatrixXd eigenMat(mat.rows, mat.cols);
  for (int i = 0; i < mat.rows; ++i) {
    for (int j = 0; j < mat.cols; ++j) {
      eigenMat(i,j) = static_cast<double>(mat.at<uchar>(i,j));
    }
  }
  return eigenMat;
}

cv::Mat eig2mat(const Eigen::MatrixXd &eig) {
  cv::Mat mat(eig.rows(), eig.cols(), CV_8UC1);

  for (int i = 0; i < eig.rows(); ++i) {
    for (int j = 0; j < eig.cols(); ++j) {
      mat.at<uchar>(i,j) = static_cast<uchar>(eig(i,j));
    }
  }
  return mat;
}


void stat_image(cv::Mat &image) {
  int rows = image.rows;
  int cols = image.cols;
  int channels = image.channels();
  std::cout << "rows:" << rows << "\ncols:" << cols <<"\nchannels:" << channels << std::endl;
  cv::imshow("Loaded image",image);
  cv::waitKey(0);
}

cv::Mat load_image(char* filename) {
  cv::Mat image = cv::imread(filename);

  if (image.empty()) {
    std::cout << "file not found" << std:: endl;
  }
  return image;
}

int main(int argc, char** argv){
  if (argc != 2) {
    std::cout << "error no filename specified" << std::endl;
    return -1;
  }
  cv::Mat image = load_image(argv[1]);
  // cv::Mat image = cv::imread(argv[1]);

  // if (image.empty()) {
  //   std::cout << "file not found" << std:: endl;
  //   return -1;
  // }
  preprocess_image(image);
  stat_image(image);
  
  Eigen::MatrixXd imgMatrix = mat2eig(image);
  std::cout << typeid(imgMatrix.row(0)).name() << std::endl;
  imgMatrix = imgMatrix.array() + 10;
  cv::Mat matrixImg = eig2mat(imgMatrix);
  stat_image(matrixImg);

  return 0;
}