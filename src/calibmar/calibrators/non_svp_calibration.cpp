#include "non_svp_calibration.h"

#include <colmap/estimators/absolute_pose.h>
#include <colmap/estimators/cost_functions.h>
#include <colmap/estimators/pose.h>
#include <colmap/geometry/pose.h>
#include <colmap/image/undistortion.h>
#include <colmap/math/matrix.h>
#include <colmap/scene/projection.h>
#include <colmap/sensor/models_refrac.h>
#include <opencv2/calib3d.hpp>

namespace {

  colmap::Camera UndistortPoints(colmap::Camera& camera, const std::vector<Eigen::Vector2d>& points_2D,
                                 std::vector<Eigen::Vector2d>& undistorted_points) {
    colmap::UndistortCameraOptions undistort_options;
    undistort_options.blank_pixels = 1.0;
    colmap::Camera undistorted_camera = colmap::UndistortCamera(undistort_options, camera);

    for (Eigen::Vector2d point : points_2D) {
      Eigen::Vector2d point_word = camera.CamFromImg(point);
      undistorted_points.push_back(undistorted_camera.ImgFromCam(point_word));
    }

    return undistorted_camera;
  }

  void ExtractOuterChessboardLines(const std::vector<Eigen::Vector2d>& corners, const std::pair<int, int>& pattern_cols_rows,
                                   std::vector<Eigen::Vector3d>& outer_lines,
                                   std::vector<std::vector<Eigen::Vector2d>>& outer_line_points) {
    int columns = pattern_cols_rows.first;
    int rows = pattern_cols_rows.second;
    size_t idx;

    std::vector<Eigen::Vector2d> outerCorners;

    // upper left
    idx = 0;
    outerCorners.push_back(corners[0]);
    // upper right
    idx = 0 * columns + columns - 1;
    outerCorners.push_back(corners[idx]);
    // bottom right
    idx = (rows - 1) * columns + columns - 1;
    outerCorners.push_back(corners[idx]);
    // bottom left
    idx = (rows - 1) * columns + 0;
    outerCorners.push_back(corners[idx]);

    std::vector<Eigen::Vector2d> oneLine;
    // compute line1
    Eigen::Vector3d pt1, pt2, line;  // homogeouns 2D image point
    pt1 = Eigen::Vector3d(outerCorners[0].x(), outerCorners[0].y(), 1.0);
    pt2 = Eigen::Vector3d(outerCorners[1].x(), outerCorners[1].y(), 1.0);
    line = pt1.cross(pt2);
    line.normalize();
    outer_lines.push_back(line);
    // push the line points into vectors

    for (size_t i = 0; i < columns; i++) {
      idx = i;
      oneLine.push_back(corners[idx]);
    }
    outer_line_points.push_back(oneLine);

    // compute line2
    pt1 = Eigen::Vector3d(outerCorners[1].x(), outerCorners[1].y(), 1.0);
    pt2 = Eigen::Vector3d(outerCorners[2].x(), outerCorners[2].y(), 1.0);
    line = pt1.cross(pt2);
    line.normalize();
    outer_lines.push_back(line);

    oneLine.clear();
    for (size_t i = 0; i < rows; i++) {
      idx = i * columns + columns - 1;
      oneLine.push_back(corners[idx]);
    }
    outer_line_points.push_back(oneLine);

    // compute line3

    pt1 = Eigen::Vector3d(outerCorners[3].x(), outerCorners[3].y(), 1.0);
    pt2 = Eigen::Vector3d(outerCorners[2].x(), outerCorners[2].y(), 1.0);
    line = pt1.cross(pt2);
    line.normalize();
    outer_lines.push_back(line);

    oneLine.clear();
    for (size_t i = 0; i < columns; i++) {
      idx = (rows - 1) * columns + i;
      oneLine.push_back(corners[idx]);
    }
    outer_line_points.push_back(oneLine);

    // compute line4
    pt1 = Eigen::Vector3d(outerCorners[0].x(), outerCorners[0].y(), 1.0);
    pt2 = Eigen::Vector3d(outerCorners[3].x(), outerCorners[3].y(), 1.0);
    line = pt1.cross(pt2);
    line.normalize();
    outer_lines.push_back(line);

    oneLine.clear();
    for (size_t i = 0; i < rows; i++) {
      idx = i * columns + 0;
      oneLine.push_back(corners[idx]);
    }
    outer_line_points.push_back(oneLine);
  }

  // Hesse normal form line transposed * point
  double DistancePointToLine(const Eigen::Vector3d& line, const Eigen::Vector2d& point2d) {
    double b = sqrt(line[0] * line[0] + line[1] * line[1]);
    double a = abs(line[0] * point2d[0] + line[1] * point2d[1] + line[2]);

    return a / b;
  }

  // if sign(line * p1) == sign(line*p2)
  bool OnTheSameSideOfLine(const Eigen::Vector3d& line, const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2) {
    double res1 = line[0] * pt1[0] + line[1] * pt1[1] + line[2];
    double res2 = line[0] * pt2[0] + line[1] * pt2[1] + line[2];
    return (res1 * res2) >= 0;
  }

  bool InvertAxisConvexityTest(const std::vector<Eigen::Vector2d>& points_2D, const std::pair<int, int> pattern_cols_rows,
                               Eigen::Vector2d refraction_center) {
    std::vector<Eigen::Vector3d> outer_lines;
    std::vector<std::vector<Eigen::Vector2d>> outer_line_points;
    ExtractOuterChessboardLines(points_2D, pattern_cols_rows, outer_lines, outer_line_points);

    // find the line which has the largest distance to the refraction center
    double distMax = std::numeric_limits<double>::min();
    size_t distMaxIdx = -1;
    for (size_t i = 0; i < outer_lines.size(); i++) {
      double dist = DistancePointToLine(outer_lines[i], refraction_center);
      if (dist > distMax) {
        distMax = dist;
        distMaxIdx = i;
      }
    }

    // check whether the points on the same side of the line
    size_t voteForward = 0;
    size_t voteBackward = 0;
    Eigen::Vector3d lineMaxDist = outer_lines[distMaxIdx];
    for (Eigen::Vector2d& pt : outer_line_points[distMaxIdx]) {
      if (OnTheSameSideOfLine(lineMaxDist, refraction_center, pt)) {
        voteForward++;
      }
      else {
        voteBackward++;
      }
    }

    if (voteForward > voteBackward && ((double)voteForward / (double)voteBackward) >= 0.8) {
      return true;
    }
    else {
      return false;
    }
  }
}

namespace calibmar {
  namespace non_svp_calibration {

    void EstimateInitialDomeOffset(const std::vector<Eigen::Vector3d>& points_3D, const std::vector<Eigen::Vector2d>& points_2D,
                                   const std::pair<int, int> pattern_cols_rows, colmap::Camera& camera,
                                   double max_expected_offset_percent) {
      if (camera.refrac_model_id != colmap::DomePort::refrac_model_id) {
        throw std::runtime_error("Non dome port camera!");
      }

      std::vector<Eigen::Vector2d> points_undistorted;
      colmap::Camera undistored_camera = UndistortPoints(camera, points_2D, points_undistorted);

      std::vector<cv::Point2d> points_cv;
      std::vector<cv::Point2d> points_chessboard_plane;
      for (size_t i = 0; i < points_undistorted.size(); i++) {
        points_cv.push_back({points_undistorted[i].x(), points_undistorted[i].y()});
        points_chessboard_plane.push_back({points_3D[i].x(), points_3D[i].y()});
      }
      // estimate the fundamental matrix based on calibration grid and the refracted corners
      cv::Mat F = cv::findFundamentalMat(points_chessboard_plane, points_cv, cv::FM_LMEDS);
      // find the left null-space of fundamental matrix F (or , the right-null space of F.t())
      cv::Mat e;
      cv::Mat b = cv::Mat::zeros(3, 1, CV_64F);
      cv::SVD::solveZ(F.t(), e);

      // transform to a point
      Eigen::Vector2d refraction_center(e.at<double>(0) / e.at<double>(2), e.at<double>(1) / e.at<double>(2));

      Eigen::Vector2d offset = undistored_camera.CamFromImg(refraction_center);
      Eigen::Vector3d offset_direction(offset.x(), offset.y(), 1.0);
      offset_direction.normalize();

      if (InvertAxisConvexityTest(points_undistorted, pattern_cols_rows, refraction_center)) {
        offset_direction = -offset_direction;
      }

      // Cx, Cy, Cz, radius
      std::vector<double>& params = camera.refrac_params;
      Eigen::Vector3d dome_center = (max_expected_offset_percent * params[3]) * offset_direction;
      params[0] = dome_center.x();
      params[1] = dome_center.y();
      params[2] = dome_center.z();
    }

    bool EstimateAbsolutePoseRefractiveCamera(const std::vector<Eigen::Vector3d>& points_3D,
                                              const std::vector<Eigen::Vector2d>& points_2D,
                                              Eigen::Quaterniond* rotation_quaternion, Eigen::Vector3d* translation,
                                              colmap::Camera* camera, bool use_initial_pose_guess) {
      colmap::Rigid3d pose(*rotation_quaternion, *translation);

      // TODO: check parametrization, and colmap function
      colmap::AbsolutePoseEstimationOptions abs_pose_options;
      abs_pose_options.ransac_options.min_num_trials = 1000;
      abs_pose_options.ransac_options.max_num_trials = 2000;
      abs_pose_options.ransac_options.confidence = 0.995;
      abs_pose_options.ransac_options.max_error = 10.0;
      abs_pose_options.estimate_focal_length = false;

      size_t num_inliers;
      std::vector<char> inlier_mask(points_2D.size(), true);

      if (!use_initial_pose_guess) {
        if (!colmap::EstimateAbsolutePose(abs_pose_options, points_2D, points_3D, &pose, camera, &num_inliers, &inlier_mask)) {
          return false;
        }
      }

      colmap::AbsolutePoseRefinementOptions abs_pose_refinement_options;
      abs_pose_refinement_options.gradient_tolerance = 1e-20;
      abs_pose_refinement_options.refine_extra_params = false;
      abs_pose_refinement_options.refine_focal_length = false;
      abs_pose_refinement_options.max_num_iterations = 500;
      abs_pose_refinement_options.print_summary = false;

      if (!colmap::RefineAbsolutePose(abs_pose_refinement_options, inlier_mask, points_2D, points_3D, &pose, camera)) {
        return false;
      }

      *rotation_quaternion = pose.rotation;
      *translation = pose.translation;

      ceres::Problem problem;

      for (size_t i = 0; i < points_2D.size(); ++i) {
        const Eigen::Vector3d& point3D = points_3D[i];

        ceres::CostFunction* cost_function = nullptr;
#define CAMERA_COMBINATION_MODEL_CASE(CameraRefracModel, CameraModel)                                                            \
  if (camera->model_id == colmap::CameraModel::model_id &&                                                                       \
      camera->refrac_model_id == colmap::CameraRefracModel::refrac_model_id) {                                                   \
    cost_function = colmap::ReprojErrorRefracCostFunction<colmap::CameraRefracModel, colmap::CameraModel>::Create(points_2D[i]); \
  }                                                                                                                              \
  else

        CAMERA_COMBINATION_MODEL_IF_ELSE_CASES

#undef CAMERA_COMBINATION_MODEL_CASE

        // Circumventing the const point3D by cast. Okay since the block is kept constant.
        problem.AddResidualBlock(cost_function, nullptr, rotation_quaternion->coeffs().data(), translation->data(),
                                 (double*)point3D.data(), camera->params.data(), camera->refrac_params.data());

        problem.SetParameterBlockConstant(point3D.data()); 

      }

      if (problem.NumResiduals() > 0) {
        // Quaternion parameterization
        rotation_quaternion->normalize();
        colmap::SetQuaternionManifold(&problem, rotation_quaternion->coeffs().data());
        // Camera parameterization.

        problem.SetParameterBlockConstant(camera->params.data());
        problem.SetParameterBlockConstant(camera->refrac_params.data());
      }

      ceres::Solver::Options solver_options;
      solver_options.gradient_tolerance = 1e-20;
      solver_options.function_tolerance = 1e-20;
      solver_options.max_num_iterations = 50000;
      solver_options.linear_solver_type = ceres::SPARSE_SCHUR;//SPARSE_SCHUR DENSE_QR 
      solver_options.minimizer_progress_to_stdout = false;
      solver_options.num_threads = std::thread::hardware_concurrency();

      ceres::Solver::Summary summary;
      ceres::Solve(solver_options, &problem, &summary);

      rotation_quaternion->normalize();

      return true;
    }
  }
}