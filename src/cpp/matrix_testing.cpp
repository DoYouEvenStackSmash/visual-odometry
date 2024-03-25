#include <iostream>
#include <eigen3/Eigen/Core>
#include <vector>
int main() {
  Eigen::MatrixXd matrix(3,4);
  matrix << 1,2,3,4,
            5,6,7,8,
            9,10,11,12;
  for (int i = 0; i < matrix.rows(); ++i) {
    for (int j = 0; j < matrix.cols(); ++j) {
      std::cout << matrix(i,j) << ",";
    }
    std::cout << std::endl;
  }
  Eigen::MatrixXd y(3,4);
  y.setZero();
  std::vector<double> h = {0.5,0.5};
  for (int r = 0; r < matrix.rows(); ++r) {
    for (int i = 0; i < matrix.cols(); ++i) {
      for (int j = 0; j < h.size(); ++j) {
        if (i - j < 0)
          continue;
        y(r,i) = y(r,i) + h[j] * matrix(r, i-j);
      }
    }
  }
  std::cout << y << std::endl;
  return 0;
}