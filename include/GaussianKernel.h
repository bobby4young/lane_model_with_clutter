#ifndef LANE_MODEL_KERNEL_H_
#define LANE_MODEL_KERNEL_H_

#include <vector>
#include <string>
#include "KernelDensityEstimation"

namespace lane_model {

typedef kde::Gaussian<double> kernelType;
typedef kde::DiagonalBandwidthMatrix<double> bandwidthType;
typedef kde::AllNeighbours<double> neighboursType;
typedef kde::KernelDensityEstimator<kernelType,bandwidthType,neighboursType> GaussianKernelDensityEstimator;
typedef typename GaussianKernelDensityEstimator::realVectorType realVectorType;
typedef typename GaussianKernelDensityEstimator::realMatrixType realMatrixType;
typedef typename GaussianKernelDensityEstimator::indexType indexType;

class GaussianKernel : public GaussianKernelDensityEstimator {
 public:
  GaussianKernel(realMatrixType const & data);
  double ComputePDF(std::vector<double> data);
};

} //ns

#endif  // LANE_MODEL_KERNEL_H_
