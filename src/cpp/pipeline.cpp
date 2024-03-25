#include "kernels.h"
#include "image_handler.h"
#include "IQR.h"

void normalize(Eigen::MatrixXd &matrix) {
  matrix = (882.5*7.5)/(matrix.array()+1);
  double minval = matrix.minCoeff();
  double maxval = matrix.maxCoeff();
  std::cout << minval << ',' << maxval << std::endl;
  matrix = (matrix.array() - minval) / (maxval - minval) * -1 + 1;
  matrix =(matrix.array()) * (maxval - minval);

}

int sign(double &a) {
  if (a==0)
    return 0;
  return a < 0 ? -1 : 1;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cout << "error no filename specified" << std::endl;
    return -1;
  }
  cv::Mat image = load_image(argv[1]);
  preprocess_image(image);
  Eigen::MatrixXd imgMatrix = mat2eig(image);
  
  int n = 5;
  int mean_delay = 9;
  int gauss_delay = 5;
  double sigma = 1;
  
  std::vector<double> gaussianKernel = generateGaussianKernel(gauss_delay, sigma);
  std::vector<double> meanKernel = generateMeanKernel(mean_delay);
  std::vector<double> derivativeKernel = generateDerivativeKernel(n);
  std::vector<double> lpf = {0,0,0,0,1,1,1,1,1,0,0,0,0};
  std::vector<std::vector<double>> pipeline = {gaussianKernel, meanKernel, derivativeKernel};
  std::vector<std::string> pipeline_names = {"gaussian","mean","derivative"};
  Eigen::MatrixXd val = imgMatrix;
  std::cout << "initial" << std::endl;
  cv::Mat m = eig2mat(val);
  stat_image(m);

  
  
  std::cout << "Normalizing" << std::endl;
  normalize(val);
  m = eig2mat(val.array()*100);
  stat_image(m);

  // val = conv(val,lpf);
  // m = eig2mat(val.array() * 100);
  // stat_image(m);


  std::cout << pipeline_names[0] << std::endl;
  val = conv(val, pipeline[0]);
  m = eig2mat(val.array()*100);
  stat_image(m);

  std::cout <<"blocking" << std::endl;
  int lb = 80;
  int ub = 300;
  int window = 300;
  Eigen::MatrixXd win_val = val.block(0,lb,val.rows(),window);
  //normalize(win_val);
  m = eig2mat(win_val);
  stat_image(m);

  std::cout << pipeline_names[1] << std::endl;
  win_val = conv(win_val, pipeline[1]);
  m = eig2mat(win_val);
  stat_image(m);

  // stat_image(m);
  std::cout << pipeline_names[2] << std::endl;
  win_val = conv(win_val, pipeline[2]);
  m = eig2mat(win_val*100);
  stat_image(m);

  std::cout << "rejecting outliers" << std::endl;
  outlier_rejection(win_val);
  m = eig2mat(win_val);
  stat_image(m);

  //normalize(win_val);

  std::cout << "Obstacles" << std::endl;
  Eigen::MatrixXd obstacles(val.rows(),window);
  obstacles.setZero();
  //obstacles = win_val;
  obstacles = obstacles.array() + 1;
  Eigen::VectorXd row_means(win_val.rows(),1);
  row_means = win_val.rowwise().mean();
  std::cout << row_means << std::endl;
  int filter_padding = lb;
  for (int i = 0; i < win_val.rows(); ++i) {
    for (int j = filter_padding; j < window-lb; ++j) {
      // std::cout << win_val(i,j) << std::endl;
      if ((sign(win_val(i,j)) != sign(win_val(i,j-1)))){//} or (abs(win_val(i,j)) < abs(row_means(i)))) {

        for (int k = j; k < obstacles.cols(); ++k) {
          obstacles(i,k) = 0;
        }
        break;
      }
    }
  }

  
  m = eig2mat(obstacles.array()*100);
  stat_image(m);
  std::vector<int> posns(obstacles.rows(),window);
  Eigen::MatrixXd endval(val.rows(), val.cols());
  endval.setZero();
  for (int i = 0; i < obstacles.rows(); ++i) {
    for (int j =0; j < obstacles.cols(); ++j) {
      if (obstacles(i,j) != 1) {
        posns[i] = j-1;
        break;
      }
    }
  }

  for (int i = 0; i < endval.rows(); ++i) {
    int j = 0;
    int rt = lb;
    for (int j = 0; j < rt + posns[i]; ++j) {
      endval(i,j) = imgMatrix(i,j);
    } 
    for (int j = rt+posns[i];j < endval.cols(); ++j) {
      if (posns[i] < window)
        endval(i,j) = imgMatrix(i,rt+posns[i]);
      else
        endval(i,j) = imgMatrix(i,j);
    }
  }
  //normalize(endval);
  m = eig2mat(endval.array()*50);
  stat_image(m);

  return 0;
}