// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/registration/registration_result.hpp>

namespace small_gicp {

/// @brief PCL registration interfaces.
template <typename PointSource, typename PointTarget>
class RegistrationPCL : public pcl::Registration<PointSource, PointTarget, float> {
public:
  using Scalar = float;
  using Matrix4 = typename pcl::Registration<PointSource, PointTarget, Scalar>::Matrix4;

  using PointCloudSource = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudSource;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudTarget;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

  using Ptr = pcl::shared_ptr<RegistrationPCL<PointSource, PointTarget>>;
  using ConstPtr = pcl::shared_ptr<const RegistrationPCL<PointSource, PointTarget>>;

protected:
  using pcl::Registration<PointSource, PointTarget, Scalar>::reg_name_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::target_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::corr_dist_threshold_;

  using pcl::Registration<PointSource, PointTarget, Scalar>::nr_iterations_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::max_iterations_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::final_transformation_;

  using pcl::Registration<PointSource, PointTarget, Scalar>::transformation_epsilon_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::converged_;

public:
  RegistrationPCL();
  virtual ~RegistrationPCL();

  /// @brief Set the number of threads to use.
  void setNumThreads(int n);
  /// @brief Set the number of neighbors for covariance estimation.
  /// @note  This is equivalent to `setNumNeighborsForCovariance`. Just exists for compatibility with pcl::GICP.
  void setCorrespondenceRandomness(int k);
  /// @brief Set the number of neighbors for covariance estimation.
  void setNumNeighborsForCovariance(int k);
  /// @brief Set the voxel resolution for VGICP.
  void setVoxelResolution(double r);
  /// @brief Set rotation epsilon for convergence check.
  void setRotationEpsilon(double eps);
  /// @brief Set registration type ("GICP" or "VGICP").
  void setRegistrationType(const std::string& type);
  /// @brief Set the verbosity flag.
  void setVerbosity(bool verbose);
  /// @brief Set the initial lambda factor for LM optimizer.
  void setInitialLambdaFactor(double init_lambda_factor);
  void setMaxCorrespondenceDistance(double distance);


  /// @brief Get the final Hessian matrix ([rx, ry, rz, tx, ty, tz]).
  const Eigen::Matrix<double, 6, 6>& getFinalHessian() const;

  /// @brief Get the detailed registration result.
  const RegistrationResult& getRegistrationResult() const;

  /// @brief  Set the input source (aligned) point cloud.
  void setInputSource(const PointCloudSourceConstPtr& cloud) override;
  /// @brief  Set the input target (fixed) point cloud.
  void setInputTarget(const PointCloudTargetConstPtr& cloud) override;

  /// @brief Register the input source (aligned) point cloud without modifying internal data structures.
  void registerInputSource(const PointCloudSourceConstPtr& cloud);
  /// @brief Register the input target (fixed) point cloud without modifying internal data structures.
  void registerInputTarget(const PointCloudTargetConstPtr& cloud);

  /// @brief Set source point covariances.
  void setSourceCovariances(std::shared_ptr<std::vector<Eigen::Matrix4d>>& covs);
  /// @brief Set target point covariances.
  void setTargetCovariances(std::shared_ptr<std::vector<Eigen::Matrix4d>>& covs);
  

  /// @brief Get source point covariances.
  std::shared_ptr<std::vector<Eigen::Matrix4d>> getSourceCovariances() const;
  /// @brief Get target point covariances.
  std::shared_ptr<std::vector<Eigen::Matrix4d>> getTargetCovariances() const;

  /// @brief Calculate source point covariances.
  void calculateSourceCovariances();
  /// @brief Calculate target point covariances.
  void calculateTargetCovariances();

  /// @brief Swap source and target point clouds and their augmented data (KdTrees, covariances, and voxelmaps).
  void swapSourceAndTarget();
  /// @brief Clear source point cloud.
  void clearSource();
  /// @brief Clear target point cloud.
  void clearTarget();

protected:
  virtual void computeTransformation(PointCloudSource& output, const Matrix4& guess) override;

public:
  double source_density_;
  double target_density_;

  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<PointSource>>> target_tree_;  ///< KdTree for target point cloud.
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<PointSource>>> source_tree_;  ///< KdTree for source point cloud.

  std::shared_ptr<std::vector<Eigen::Matrix4d>> target_covs_;  ///< Covariances of target points
  std::shared_ptr<std::vector<Eigen::Matrix4d>> source_covs_;  ///< Covariances of source points.
  
protected:
  int num_threads_;                ///< Number of threads to use.
  int k_correspondences_;          ///< Number of neighbors for covariance estimation.
  double rotation_epsilon_;        ///< Rotation epsilon for convergence check.
  double voxel_resolution_;        ///< Voxel resolution for VGICP.
  std::string registration_type_;  ///< Registration type ("GICP" or "VGICP").
  bool verbose_;                   ///< Verbosity flag.

  double lm_init_lambda_factor_;  ///< Initial lambda factor for LM optimizer.
  double lm_lambda_;               ///< Current lambda value for LM optimizer.

  std::shared_ptr<GaussianVoxelMap> target_voxelmap_;  ///< VoxelMap for target point cloud.
  std::shared_ptr<GaussianVoxelMap> source_voxelmap_;  ///< VoxelMap for source point cloud.

  RegistrationResult result_;  ///< Detailed registration result.
};

}  // namespace small_gicp

#include <small_gicp/pcl/pcl_registration_impl.hpp>