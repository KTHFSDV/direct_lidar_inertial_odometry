#ifndef FAST_GICP_FAST_VGICP_CUDA_IMPL_HPP
#define FAST_GICP_FAST_VGICP_CUDA_IMPL_HPP

#include <atomic>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/registration.h>

#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#include <fast_gicp/cuda/fast_vgicp_cuda.cuh>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

namespace fast_gicp {

template<typename PointSource, typename PointTarget>
FastVGICPCuda<PointSource, PointTarget>::FastVGICPCuda() : LsqRegistration<PointSource, PointTarget>() {
  this->reg_name_ = "FastVGICPCuda";
  k_correspondences_ = 20;
  voxel_resolution_ = 1.0;
  regularization_method_ = RegularizationMethod::NONE;
  neighbor_search_method_ = NearestNeighborMethod::CPU_PARALLEL_KDTREE;

  vgicp_cuda_.reset(new cuda::FastVGICPCudaCore());
  vgicp_cuda_->set_resolution(voxel_resolution_);
  vgicp_cuda_->set_kernel_params(0.5, 3.0);
}

template<typename PointSource, typename PointTarget>
FastVGICPCuda<PointSource, PointTarget>::~FastVGICPCuda() {}

template<typename PointSource, typename PointTarget>
void FastVGICPCuda<PointSource, PointTarget>::setCorrespondenceRandomness(int k) {}

template<typename PointSource, typename PointTarget>
void FastVGICPCuda<PointSource, PointTarget>::setResolution(double resolution) {
  vgicp_cuda_->set_resolution(resolution);
}

template <typename PointSource, typename PointTarget>
void FastVGICPCuda<PointSource, PointTarget>::setKernelWidth(double kernel_width, double max_dist) {
  if (max_dist <= 0.0) {
    max_dist = kernel_width * 5.0;
  }
  vgicp_cuda_->set_kernel_params(kernel_width, max_dist);
}

template<typename PointSource, typename PointTarget>
void FastVGICPCuda<PointSource, PointTarget>::setRegularizationMethod(RegularizationMethod method) {
  regularization_method_ = method;
}

template <typename PointSource, typename PointTarget>
void FastVGICPCuda<PointSource, PointTarget>::setNeighborSearchMethod(NeighborSearchMethod method, double radius) {
  vgicp_cuda_->set_neighbor_search_method(method, radius);
}

template <typename PointSource, typename PointTarget>
void FastVGICPCuda<PointSource, PointTarget>::setNearestNeighborSearchMethod(NearestNeighborMethod method) {
  neighbor_search_method_ = method;
}

template<typename PointSource, typename PointTarget>
void FastVGICPCuda<PointSource, PointTarget>::swapSourceAndTarget() {
  vgicp_cuda_->swap_source_and_target();
  input_.swap(target_);
}

template<typename PointSource, typename PointTarget>
void FastVGICPCuda<PointSource, PointTarget>::clearSource() {
  input_.reset();
}

template<typename PointSource, typename PointTarget>
void FastVGICPCuda<PointSource, PointTarget>::clearTarget() {
  target_.reset();
}

template<typename PointSource, typename PointTarget>
void FastVGICPCuda<PointSource, PointTarget>::setInputSource(const PointCloudSourceConstPtr& cloud) {
  // the input cloud is the same as the previous one
  if(cloud == input_) {
    return;
  }

  pcl::Registration<PointSource, PointTarget, Scalar>::setInputSource(cloud);

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points(cloud->size());
  std::transform(cloud->begin(), cloud->end(), points.begin(), [=](const PointSource& pt) { return pt.getVector3fMap(); });

  vgicp_cuda_->set_source_cloud(points);
  switch(neighbor_search_method_) {
    case NearestNeighborMethod::CPU_PARALLEL_KDTREE: {
      std::vector<int> neighbors = find_neighbors_parallel_kdtree<PointSource>(k_correspondences_, cloud);
      vgicp_cuda_->set_source_neighbors(k_correspondences_, neighbors);
      vgicp_cuda_->calculate_source_covariances(regularization_method_);
      
    } break;
    case NearestNeighborMethod::GPU_BRUTEFORCE:
      vgicp_cuda_->find_source_neighbors(k_correspondences_);
      vgicp_cuda_->calculate_source_covariances(regularization_method_);
      break;
    case NearestNeighborMethod::GPU_RBF_KERNEL:
      vgicp_cuda_->calculate_source_covariances_rbf(regularization_method_);
      break;
  }
}

template <typename PointSource, typename PointTarget>
void FastVGICPCuda<PointSource, PointTarget>::registerInputSource(const PointCloudSourceConstPtr& cloud) {
  if (input_ == cloud) {
    return;
  }
  pcl::Registration<PointSource, PointTarget, Scalar>::setInputSource(cloud);
}

template <typename PointSource, typename PointTarget>
void FastVGICPCuda<PointSource, PointTarget>::registerInputTarget(const PointCloudTargetConstPtr& cloud) {
  if (target_ == cloud) {
    return;
  }
  pcl::Registration<PointSource, PointTarget, Scalar>::setInputTarget(cloud);
}

// TODO : REFACTOR THIS TO USE GPU
template <typename PointSource, typename PointTarget>
bool FastVGICPCuda<PointSource, PointTarget>::calculateSourceCovariances() {
  std::cout << "calculateSourceCovariances" << std::endl;
  std::shared_ptr<CovarianceList> source_covs = std::make_shared<CovarianceList>();
  std::shared_ptr<float> source_density = std::make_shared<float>();
  bool ret = calculate_covariances(input_, *source_kdtree_, *source_covs, *source_density);
  // CovarianceList cov;
  // vgicp_cuda_->get_source_covariances(cov);
  // *source_covs_ = cov.cast<double>();
  source_covs_ = source_covs;
  source_density_ = *source_density;
  std::cout << "calculateSourceCovariances finished" << std::endl;
  return ret;
}

// TODO : REFACTOR THIS TO USE GPU
template <typename PointSource, typename PointTarget>
bool FastVGICPCuda<PointSource, PointTarget>::calculateTargetCovariances() {
  std::cout << "calculateTargetCovariances" << std::endl;
  std::shared_ptr<CovarianceList> target_covs = std::make_shared<CovarianceList>();
  std::shared_ptr<float> target_density = std::make_shared<float>();
  bool ret = calculate_covariances(target_, *target_kdtree_, *target_covs, *target_density);
  target_covs_ = target_covs;
  target_density_ = *target_density;
  std::cout << "calculateTargetCovariances finished" << std::endl;
  return ret;
}


// TODO : THIS SHOULD BE REMOVED ALTOGETHER
// So instead you call calculate_source_covariances through the vgicp_cuda
template <typename PointSource, typename PointTarget>
template <typename PointT>
bool FastVGICPCuda<PointSource, PointTarget>::calculate_covariances(
  const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
  const pcl::search::KdTree<PointT>& kdtree,
  CovarianceList& covariances,
  float& density) {

  std::cout << "calculate_covariances" << std::endl;

//   covariances.resize(cloud->size());
//   float sum_k_sq_distances = 0.0;

//   std::cout << "step 1 done, starting pragma for loop " << std::endl;

//   std::cout << "num_threads_: " << num_threads_ << std::endl;

// #pragma omp parallel for num_threads(1) schedule(guided, 1) reduction(+:sum_k_sq_distances)
//   for (int i = 0; i < cloud->size(); i++) {
//     std::cout << "i: " << i << std::endl;
//     std::vector<int> k_indices;
//     std::vector<float> k_sq_distances;
//     kdtree.nearestKSearch(cloud->at(i), k_correspondences_, k_indices, k_sq_distances);

//     std::cout << "for loop 1" << std::endl;

//     const int normalization = ((k_correspondences_ - 1) * (2 + k_correspondences_)) / 2;
//     sum_k_sq_distances += std::accumulate(k_sq_distances.begin()+1, k_sq_distances.end(), 0.0) / normalization;

//     std::cout << "for loop 2" << std::endl;

//     Eigen::Matrix<double, 4, -1> neighbors(4, k_correspondences_);
//     for (int j = 0; j < k_indices.size(); j++) {
//       neighbors.col(j) = cloud->at(k_indices[j]).getVector4fMap().template cast<double>();
//     }

//     std::cout << "for loop 3" << std::endl;

//     neighbors.colwise() -= neighbors.rowwise().mean().eval();
//     Eigen::Matrix4d cov = neighbors * neighbors.transpose() / k_correspondences_;

//     std::cout << "for loop 4" << std::endl;

//     if (regularization_method_ == RegularizationMethod::NONE) {
//       covariances[i] = cov;
//     } else if (regularization_method_ == RegularizationMethod::FROBENIUS) {
//       double lambda = 1e-3;
//       Eigen::Matrix3d C = cov.block<3, 3>(0, 0).cast<double>() + lambda * Eigen::Matrix3d::Identity();
//       Eigen::Matrix3d C_inv = C.inverse();
//       covariances[i].setZero();
//       covariances[i].template block<3, 3>(0, 0) = (C_inv / C_inv.norm()).inverse();
//     } else {
//       Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
//       Eigen::Vector3d values;

//       switch (regularization_method_) {
//         default:
//           std::cerr << "here must not be reached" << std::endl;
//           abort();
//         case RegularizationMethod::PLANE:
//           values = Eigen::Vector3d(1, 1, 1e-3);
//           break;
//         case RegularizationMethod::MIN_EIG:
//           values = svd.singularValues().array().max(1e-3);
//           break;
//         case RegularizationMethod::NORMALIZED_MIN_EIG:
//           values = svd.singularValues() / svd.singularValues().maxCoeff();
//           values = values.array().max(1e-3);
//           break;
//       }

//       covariances[i].setZero();
//       covariances[i].template block<3, 3>(0, 0) = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
//     }
//     std::cout << "i: " << i << " done" << std::endl;
//   }

//   density = sum_k_sq_distances / cloud->size();

//   std::cout << "calculate_covariances finished" << std::endl;

  return true;
}


template<typename PointSource, typename PointTarget>
void FastVGICPCuda<PointSource, PointTarget>::setInputTarget(const PointCloudTargetConstPtr& cloud) {
  // the input cloud is the same as the previous one
  if(cloud == target_) {
    return;
  }

  pcl::Registration<PointSource, PointTarget, Scalar>::setInputTarget(cloud);

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points(cloud->size());
  std::transform(cloud->begin(), cloud->end(), points.begin(), [=](const PointTarget& pt) { return pt.getVector3fMap(); });

  vgicp_cuda_->set_target_cloud(points);
  switch(neighbor_search_method_) {
    case NearestNeighborMethod::CPU_PARALLEL_KDTREE: {
      std::vector<int> neighbors = find_neighbors_parallel_kdtree<PointTarget>(k_correspondences_, cloud);
      vgicp_cuda_->set_target_neighbors(k_correspondences_, neighbors);
      vgicp_cuda_->calculate_target_covariances(regularization_method_);
    } break;
    case NearestNeighborMethod::GPU_BRUTEFORCE:
      vgicp_cuda_->find_target_neighbors(k_correspondences_);
      vgicp_cuda_->calculate_target_covariances(regularization_method_);
      break;
    case NearestNeighborMethod::GPU_RBF_KERNEL:
      vgicp_cuda_->calculate_target_covariances_rbf(regularization_method_);
      break;
  }
  vgicp_cuda_->create_target_voxelmap();
}

template <typename PointSource, typename PointTarget>
void FastVGICPCuda<PointSource, PointTarget>::setTargetCovariances(const std::shared_ptr<const CovarianceList>& covs) {
  target_covs_ = covs;
}

template<typename PointSource, typename PointTarget>
void FastVGICPCuda<PointSource, PointTarget>::computeTransformation(PointCloudSource& output, const Matrix4& guess) {
  vgicp_cuda_->set_resolution(this->voxel_resolution_);

  LsqRegistration<PointSource, PointTarget>::computeTransformation(output, guess);
}

template<typename PointSource, typename PointTarget>
template<typename PointT>
std::vector<int> FastVGICPCuda<PointSource, PointTarget>::find_neighbors_parallel_kdtree(int k, typename pcl::PointCloud<PointT>::ConstPtr cloud) const {
  pcl::search::KdTree<PointT> kdtree;
  kdtree.setInputCloud(cloud);
  std::vector<int> neighbors(cloud->size() * k);

#pragma omp parallel for schedule(guided, 8)
  for(int i = 0; i < cloud->size(); i++) {
    std::vector<int> k_indices;
    std::vector<float> k_sq_distances;
    kdtree.nearestKSearch(cloud->at(i), k, k_indices, k_sq_distances);

    std::copy(k_indices.begin(), k_indices.end(), neighbors.begin() + i * k);
  }

  return neighbors;
}

template<typename PointSource, typename PointTarget>
double FastVGICPCuda<PointSource, PointTarget>::linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) {
  vgicp_cuda_->update_correspondences(trans);
  return vgicp_cuda_->compute_error(trans, H, b);
}

template<typename PointSource, typename PointTarget>
double FastVGICPCuda<PointSource, PointTarget>::compute_error(const Eigen::Isometry3d& trans) {
  return vgicp_cuda_->compute_error(trans, nullptr, nullptr);
}

}  // namespace fast_gicp

#endif
