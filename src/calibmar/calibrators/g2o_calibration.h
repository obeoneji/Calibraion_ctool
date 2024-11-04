#pragma once
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <stdint.h>

#include <cassert>
#include <iostream>
#include <unordered_set>

#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "calibmar/core/calibration.h"
#include "calibmar/core/camera_models.h"
#include <colmap/scene/camera.h>
namespace calibmar::g2o_calibration {
  void testG2o(Calibration& calibration,int cam_num, int root_cam,std::vector<std::vector<Eigen::Vector2d>>&Cam_point2dSets,
    std::vector<std::vector<Eigen::Vector3d>>&Cam_point3dSets,std::vector<std::vector<int>>&Cam_charucoidSets,std::vector<Eigen::Vector3d>&point3d_dict,
    std::vector<int>point3d_dict_ids,std::vector<cv::Mat>&rotations,std::vector<cv::Mat>& translations,int solver_index);
}