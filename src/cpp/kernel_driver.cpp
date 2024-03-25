#include "kernels.h"
#include <vector>
#include <iostream>

int main() {
  int n = 9;
  double sigma = 1.0;

  std::vector<double> gaussianKernel = generateGaussianKernel(n, sigma);
  std::vector<double> meanKernel = generateMeanKernel(n);
  std::vector<double> derivativeKernel = generateDerivativeKernel(n);
  std::vector<std::vector<double>> pipeline = {gaussianKernel, meanKernel, derivativeKernel};
  std::vector<std::string> pipeline_names = {"gaussian","mean","derivative"};


  //std::cout << "Generated Gaussian kernel: " << std::endl;

  for (int i = 0; i < pipeline.size(); ++i) {
    std::cout << pipeline_names[i] << ":";
    for (int j = 0; j < pipeline[i].size(); ++j) {
      std::cout << pipeline[i][j] << " ";    
    }
    std::cout << std::endl;
  }
  
  std::cout << std::endl;
  return 0;
}