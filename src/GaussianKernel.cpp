#include <iostream>
#include <vector>
#include <string>
#include "GaussianKernel.h"

namespace lane_model {

GaussianKernel::GaussianKernel(realMatrixType const & data): GaussianKernelDensityEstimator(data) {}

double GaussianKernel::ComputePDF(std::vector<double> data) {
  realVectorType sample(data.size());
  for (int i = 0; i < data.size(); i++) {
    sample(i) = data[i];
  }
  return computePDF(sample);
}
} //ns