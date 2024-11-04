#pragma once

#include <colmap/feature/types.h>
#include <colmap/scene/image.h>
#include <unordered_map>

namespace calibmar {
  // Image is a single view of a calibration and holds e.g. 2D-3D corresponence and pose information.
  class Image {
   public:
    inline const std::string& Name() const;
    inline void SetName(const std::string& name);

    inline size_t AddPoint2D(const Eigen::Vector2d& point);
    inline size_t AddPoint2D_withboard(const Eigen::Vector2d& point,int j);

    inline void SetPoints2D(const std::vector<Eigen::Vector2d>& points);
    inline void SetPoints2D_withboard(const std::vector<Eigen::Vector2d>& points,int j);

    inline void SetPoints3D(const std::vector<Eigen::Vector3d>& points);
    inline void SetPoints3D_withboard(const std::vector<std::vector< Eigen::Vector3d>>& points);

    inline void SetCharucoIds(const std::vector<int> charucoIds);//set 
    inline void SetCharucoIds_withboard(const std::vector<int> charucoIds,int j);

    inline void Setboardindex(int j);

    inline const std::vector<Eigen::Vector2d>& Points2D() const;
    inline const std::vector<std::vector<Eigen::Vector2d>>& Points2D_board() const;

    inline const std::vector<Eigen::Vector3d>& Points3D() const;

    inline Eigen::Vector2d Point2D(size_t idx) const;
    inline Eigen::Vector2d Point2D_board(size_t idx,int board_index) const;


    inline const std::vector<int> &Board() const;
    inline const std::vector<int> &CharucoIds() const;
    inline const std::vector<std::vector<int>> &CharucoIds_board() const;

    inline void SetPoint3DforPoint2D(const uint32_t point3D_id, const size_t point2D_idx);
    inline void SetPoint3DforPoint2D_withboard(const uint32_t point3D_id, const size_t point2D_idx,int j);

    // Correspondences of image 2D point indices to calibration 3D point ids
    inline const std::unordered_map<size_t, uint32_t>& Correspondences() const;
    inline const std::vector<std::unordered_map<size_t, uint32_t>>& Correspondences_board() const;
    inline void ClearCorrespondences();
    inline void ClearCorrespondences_withboard();
    // Pose which is defined as the transformation from world to camera space.
    inline const colmap::Rigid3d& Pose() const;
    inline colmap::Rigid3d& Pose();
    inline void SetPose(const colmap::Rigid3d& pose);

    inline void SetDescriptors(const colmap::FeatureDescriptors& descriptors);
    inline void SetKeypoints(const colmap::FeatureKeypoints& keypoints);
    inline void SetArucoKeypoints(const std::map<int, std::vector<Eigen::Vector2d>>& aruco_keypoints);

    // sift feature descriptors, only used with 3D reconstruction
    inline const colmap::FeatureDescriptors& FeatureDescriptors() const;
    inline colmap::FeatureDescriptors& FeatureDescriptors();
    // corresponding sift feature keypoints, only used with 3D reconstruction
    inline const colmap::FeatureKeypoints& FeatureKeypoints() const;
    inline colmap::FeatureKeypoints& FeatureKeypoints();
    // Map of aruco ids and corresponding point observations, only used with aruco 3D reconstruction
    inline const std::map<int, std::vector<Eigen::Vector2d>>& ArucoKeypoints() const;
    inline void issystem();
    inline bool System() const;
    colmap::FeatureDescriptors feature_descriptors_;
    colmap::FeatureKeypoints feature_keypoints_;
    // this is a ordered map so the position of detected corners is stable regarding their position in the overall map
    std::map<int, std::vector<Eigen::Vector2d>> aruco_keypoints_;
    std::string name_;
    std::vector<Eigen::Vector2d> points2D_;
    std::vector<std::vector<Eigen::Vector2d>>points2D_board;
    std::vector<Eigen::Vector3d> points3D_;
    std::vector<std::vector<Eigen::Vector3d>>points3D_board;
    std::vector<int> charucoIds_;
    std::vector<int> boards_;
    std::vector<std::vector<int>> charucoIds_board;

    std::unordered_map<size_t, uint32_t> correspondences_;
    std::vector<std::unordered_map<size_t, uint32_t>> correspondences_board;
    colmap::Rigid3d pose_;
    bool system=false;
  };

  ////////////////////////////////////////////////////////////////////////////////
  // Implementation
  ////////////////////////////////////////////////////////////////////////////////

  inline size_t Image::AddPoint2D(const Eigen::Vector2d& point) {
    points2D_.push_back(point);
    return points2D_.size() - 1;
  }
  inline size_t Image::AddPoint2D_withboard(const Eigen::Vector2d& point,int j) {
    if (points2D_board.size() <= j){
      points2D_board.resize(j + 1);
    }
    points2D_board[j].push_back(point);
    return points2D_board[j].size() - 1;
  }

  inline void Image::issystem() {
    system=true;
  }
  inline bool Image::System()const {
    return system;
  }

  inline void Image::SetPoints2D(const std::vector<Eigen::Vector2d>& points) {
    points2D_ = points;
  }
  inline void Image::SetPoints2D_withboard(const std::vector<Eigen::Vector2d>& points,int j) {
    points2D_board[j] = points;
  }
  inline void Image::SetPoints3D(const std::vector<Eigen::Vector3d>& points) {
    points3D_ = points;
  }
  inline void Image::SetPoints3D_withboard(const std::vector<std::vector<Eigen::Vector3d>>& points) {
    points3D_board = points;
  }
  inline Eigen::Vector2d Image::Point2D(size_t idx) const {
    return points2D_.at(idx);
  }
  inline Eigen::Vector2d Image::Point2D_board(size_t idx,int board_index) const {
    return points2D_board.at(board_index).at(idx);
  }
  inline const std::vector<Eigen::Vector2d>& Image::Points2D() const {
    return points2D_;
  }
  inline const std::vector<std::vector<Eigen::Vector2d>>& Image::Points2D_board() const {
    return points2D_board;
  }
  inline void Image::SetCharucoIds(const std::vector<int> charucoIds) {
    charucoIds_ = charucoIds;
  }
  inline void Image::SetCharucoIds_withboard(const std::vector<int> charucoIds,int j) {
    if (charucoIds_board.size() <= j){
      charucoIds_board.resize(j + 1);
    }   
    charucoIds_board[j]= charucoIds;
  }
  inline void Image::Setboardindex(int j) {
    boards_.push_back(j);
  }
  inline const std::vector<int>& Image::Board() const {
    return boards_;
  }

  inline const std::vector<int>& Image::CharucoIds() const {
    return charucoIds_;
  }
  inline const std::vector<std::vector<int>>& Image::CharucoIds_board() const {
    return charucoIds_board;
  }

  inline const std::vector<Eigen::Vector3d>& Image::Points3D() const {
    return points3D_;
  }
  inline void Image::SetPoint3DforPoint2D(const uint32_t point3D_id, const size_t point2D_idx) {
    correspondences_[point2D_idx] = point3D_id;
  }
  inline void Image::SetPoint3DforPoint2D_withboard(const uint32_t point3D_id, const size_t point2D_idx,int j) {
    if (correspondences_board.size() <= j){
      correspondences_board.resize(j + 1);
    }
    correspondences_board[j][point2D_idx] = point3D_id;
  }

  inline const std::unordered_map<size_t, uint32_t>& Image::Correspondences() const {
    return correspondences_;
  }
  inline const std::vector<std::unordered_map<size_t, uint32_t>>& Image::Correspondences_board() const {
    return correspondences_board;
  }

  inline void Image::ClearCorrespondences() {
    correspondences_.clear();
  }
  inline void Image::ClearCorrespondences_withboard() {
    correspondences_board.clear();
  }
  inline const std::string& Image::Name() const {
    return name_;
  }

  inline void Image::SetName(const std::string& name) {
    name_ = name;
  }

  inline const colmap::Rigid3d& Image::Pose() const {
    return pose_;
  }

  inline colmap::Rigid3d& Image::Pose() {
    return pose_;
  }

  inline void Image::SetPose(const colmap::Rigid3d& pose) {
    pose_ = pose;
  }

  inline void Image::SetDescriptors(const colmap::FeatureDescriptors& descriptors) {
    feature_descriptors_ = descriptors;
  }

  inline void Image::SetKeypoints(const colmap::FeatureKeypoints& keypoints) {
    feature_keypoints_ = keypoints;
  }

  inline void Image::SetArucoKeypoints(const std::map<int, std::vector<Eigen::Vector2d>>& aruco_keypoints) {
    aruco_keypoints_ = aruco_keypoints;
  }

  inline const colmap::FeatureDescriptors& Image::FeatureDescriptors() const {
    return feature_descriptors_;
  }

  inline colmap::FeatureDescriptors& Image::FeatureDescriptors() {
    return feature_descriptors_;
  }

  inline const colmap::FeatureKeypoints& Image::FeatureKeypoints() const {
    return feature_keypoints_;
  }

  inline colmap::FeatureKeypoints& Image::FeatureKeypoints() {
    return feature_keypoints_;
  }

  inline const std::map<int, std::vector<Eigen::Vector2d>>& Image::ArucoKeypoints() const {
    return aruco_keypoints_;
  }
}