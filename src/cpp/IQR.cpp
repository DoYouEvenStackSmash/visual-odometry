#include "IQR.h"

std::vector<double> calculateQ1Q3(Eigen::VectorXd row) {
  std::sort(row.data(), row.data() + row.size());
  std::vector<double> ans = {row(static_cast<int>(row.size()) / 4),row(3 * static_cast<int>(row.size()) / 4)};
  return ans;
}

Eigen::MatrixXd calculateIQR(const Eigen::MatrixXd& matrix) {
  Eigen::MatrixXd iqr(matrix.rows(),2);
  for (int i = 0; i < matrix.rows(); ++i) {
    std::vector<double> ans = calculateQ1Q3(matrix.row(i));
    iqr(i,0) = ans[0];
    iqr(i,1) = ans[1]; 
  }
  return iqr;
}

void outlier_rejection(Eigen::MatrixXd& matrix,double threshold) {
  Eigen::MatrixXd bounds = calculateIQR(matrix);
  for (int i = 0; i < matrix.rows(); ++i) {
    for (int j = 0; j < matrix.cols(); ++j) {
      double thres = (bounds(i,1) - bounds(i,0)) * threshold;
      double lower_bound = bounds(i,0) - thres;
      double upper_bound = bounds(i,1) + thres;
      matrix(i,j) = matrix(i,j) < lower_bound ? lower_bound : matrix(i,j);
      matrix(i,j) = matrix(i,j) > upper_bound ? upper_bound : matrix(i,j);
    }
  }
}