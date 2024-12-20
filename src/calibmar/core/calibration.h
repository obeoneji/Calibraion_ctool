#pragma once

#include "calibmar/core/image.h"
#include <opencv2/core/eigen.hpp>
#include <colmap/scene/camera.h>
#include <colmap/scene/reconstruction.h>
#include <map>
#include <optional>

namespace calibmar {
  // Calibration holds all information about a single calibration.
  class Calibration {
   public:
    colmap::Camera& Camera();
    const colmap::Camera& Camera() const;
    colmap::Camera& Camera(size_t i);    
    const colmap::Camera& Camera(size_t i) const;
    colmap::Camera& Camera_undistorted(size_t i);    
    const colmap::Camera& Camera_undistorted(size_t i) const;
    
    void SetCamera(const colmap::Camera& camera);
    void addcamnum(const size_t cam_count);
    class Image& Image(size_t image_idx);
    const class Image& Image(size_t image_idx) const;
    class Image& Image_cam(const size_t image_idx,int i);
    class Image& Image_cam_undistorted(const size_t image_idx,int i);
    size_t AddImage(const class Image& image);
    size_t AddImage_cam(const class Image& image,int i);
    size_t AddImage_cam_undistorted(const class Image& image,int i);
    Eigen::Vector3d& Point3D(uint32_t point_id);
    const Eigen::Vector3d& Point3D(uint32_t point_id) const;
    uint32_t AddPoint3D(const Eigen::Vector3d& xyz);
    inline void issystem();
    inline bool System() const;
    inline void isg2o(bool flag,int solver_index);
    inline bool G2O_flag()const;
    inline bool G2O_solver_flag()const;
    inline void recomputeflag();
    inline bool Recomputeflag() const; 
    std::map<uint32_t, Eigen::Vector3d>& Points3D();
    const std::map<uint32_t, Eigen::Vector3d>& Points3D() const;
    void SetPoints3D(const std::map<uint32_t, Eigen::Vector3d>& points3D);
    void SetPoints3D_withboard(const std::map<uint32_t,std::map<uint32_t, Eigen::Vector3d>>& points3D_map);

    std::vector<class Image>& Images();
    const std::vector<class Image>& Images() const;
    std::vector<class Image>& Cam_Images(int i);
    const std::vector<class Image>& Cam_Images(int i) const;
    std::vector<class Image>& Cam_Images_Undistorted(int i);
    const std::vector<class Image>& Cam_Images_Undistorted(int i) const;
    double CalibrationRms() const;
    void SetCalibrationRms(double rms);

    double Projectionerr_before() const;

    void Setprojectionerr_before(double error);

    double Projectionerr_ba() const;
    void Setprojectionerr_ba(double error);
    std::vector<double>& IntrinsicsStdDeviations();
    const std::vector<double>& IntrinsicsStdDeviations() const;
    void SetIntrinsicsStdDeviations(const std::vector<double>& intrinsincs_std);

    std::vector<double>& HousingParamsStdDeviations();
    const std::vector<double>& HousingParamsStdDeviations() const;
    void SetHousingParamsStdDeviations(const std::vector<double>& housing_std);


    void SetInitalPose(const std::vector<cv::Mat>&Cam_rotations,const std::vector<cv::Mat>&Cam_translations);
    cv::Mat & InitalPose(int i);
    const cv::Mat & InitalPose(int i)const;

    void SetbaPose(const std::vector<cv::Mat>&Cam_rotations_afterba,const std::vector<cv::Mat>&Cam_translations_afterba);
    cv::Mat & BaPose(int i);
    const cv::Mat & BaPose(int i)const;


    std::vector<double>& PerViewRms();
    const std::vector<double>& PerViewRms() const;
    void SetPerViewRms(const std::vector<double>& housing_std);

    std::vector<int>& PerView3DPointCount();
    const std::vector<int>& PerView3DPointCount() const;
    void SetPerView3DPoints(const std::vector<int>& per_view_3d_point_count);

    // Set infos about the used calibration target. Used in report generation.
    void SetCalibrationTargetInfo(const std::string& info);
    // Get infos about the used calibration target. Used in report generation.
    const std::string& GetCalibrationTargetInfo() const;

    // Contains the stereo pose in case this is the calibration of a stereo camera.
    // That is: a 3D point X_C in camera coordinates is transformed to a 3D point X_W in world coordinates by X_W = R * X_C + t
    const std::optional<colmap::Rigid3d>& CameraToWorldStereo() const;
    void SetCameraToWorldStereo(const std::optional<colmap::Rigid3d>& pose);

    void GetCorrespondences(std::vector<std::vector<Eigen::Vector2d>>& points2D,
                            std::vector<std::vector<Eigen::Vector3d>>& points3D);
    void GetCorrespondences_board(std::vector<std::vector<Eigen::Vector2d>>& points2D,std::vector<std::vector<Eigen::Vector3d>>& points3D,int m);
    void GetCorrespondences_board_undistorted(std::vector<std::vector<Eigen::Vector2d>>& points2D,std::vector<std::vector<Eigen::Vector3d>>& points3D,std::vector<int>&charucoidSets,std::vector<int>& boardindexSets,int m);
    void InitializeFromReconstruction(const colmap::Reconstruction& reconstruction);
    int cam_num;
    int cam_index;

    inline int GetCamIndex() const {
    return cam_index;
    }
    int detected_board=0;
   private:
    colmap::Camera camera_;
    std::vector<colmap::Camera> cameras_;
    std::vector<colmap::Camera> cameras_undistorted_;
    double calibration_rms_ = 0;
    double projectionerr_before_ =0;
    double projectionerr_ba_ =0; 
    std::string calibration_target_info_;
    // std deviations are only available if the calibrator supports it
    std::vector<cv::Mat> initalpose;
    std::vector<cv::Mat> bapose;
    std::vector<double> intrinsics_std_deviations_;
    std::vector<double> housing_params_std_deviations_;
    std::vector<int> cam_detected_board;
    std::vector<int> cam_detected_board_undistorted;
    std::vector<class Image> images_;
    std::vector<std::vector<class Image>> cam_images_;
    std::vector<std::vector<class Image>> cam_images_undistorted_;
    std::vector<double> per_view_rms_;
    // per view 3D count only relevant for 3D target calibration, where not all points are visible in every view
    std::vector<int> per_view_3d_point_count_;
    // these 3d points are currently not used for 3d target calibration (everything is handled inside the calibrator i.e. colmap)
    std::map<uint32_t, Eigen::Vector3d> points3D_;
    std::map<uint32_t, std::map<uint32_t, Eigen::Vector3d>> points3D_map_;
    // used to generate point id
    uint32_t number_of_points3D = 0;
    // only used with stereo calibration
    std::optional<colmap::Rigid3d> stereo_pose_ = {};
    bool system=false;
    bool recompute=false;
    bool g2o_flag=false;
    int g2o_solver=0;
  };

  ////////////////////////////////////////////////////////////////////////////////
  // Implementation
  ////////////////////////////////////////////////////////////////////////////////
  inline void Calibration::issystem() {
    system=true;
  }
  inline bool Calibration::System()const {
    return system;
  }
  inline void Calibration::isg2o(bool flag,int solver_index) {
    g2o_flag=flag;
    g2o_solver=solver_index;
  }
  inline bool Calibration::G2O_flag()const {
    return g2o_flag;
  }
  inline bool Calibration::G2O_solver_flag()const {
    return g2o_solver;
  }
  inline void Calibration::recomputeflag() {
    recompute=true;
  }
  inline bool Calibration::Recomputeflag()const {
    return recompute;
  }
  inline colmap::Camera& Calibration::Camera() {
    return camera_;
    cam_index+=1;
  }

  inline const colmap::Camera& Calibration::Camera() const {
    return camera_;
  }
  inline colmap::Camera& Calibration::Camera(size_t i) {
    return cameras_.at(i);
  }
  inline const colmap::Camera& Calibration::Camera(size_t i) const{
    return cameras_.at(i);
  }
  inline colmap::Camera& Calibration::Camera_undistorted(size_t i) {
    return cameras_undistorted_.at(i);
  }
  inline const colmap::Camera& Calibration::Camera_undistorted(size_t i) const{
    return cameras_undistorted_.at(i);
  }

  inline void  Calibration::addcamnum(const size_t cam_count){
    cam_num=cam_count;
    cam_images_.resize(cam_count);
    cam_images_undistorted_.resize(cam_count);
    cam_detected_board.resize(cam_count,0);
    cam_detected_board_undistorted.resize(cam_count,0);
    cameras_.resize(cam_count);
    cameras_undistorted_.resize(cam_count);
    initalpose.resize(cam_count);
    bapose.resize(cam_count);
  }
  inline void Calibration::SetCamera(const colmap::Camera& camera) {
    camera_ = camera;
  }

  inline class Image& Calibration::Image(const size_t image_idx) {
    return images_.at(image_idx);
  }
  inline class Image& Calibration::Image_cam(const size_t image_idx,int i) {
    return cam_images_.at(i).at(image_idx);
  }
  inline class Image& Calibration::Image_cam_undistorted(const size_t image_idx,int i) {
    return cam_images_undistorted_.at(i).at(image_idx);
  }
  inline const class Image& Calibration::Image(const size_t image_idx) const {
    return images_.at(image_idx);
  }

  inline size_t Calibration::AddImage(const class Image& image) {
    images_.push_back(image);
    int s=0;
    for(int i=0;i<image.Points2D_board().size();i++)
    {
      if(image.Points2D_board()[i].size()!=0)
      {
        s++;
      }
    }
    detected_board+=s;
    return images_.size() - 1;
  }
  inline size_t Calibration::AddImage_cam(const class Image& image,int i) {
    if (cam_images_[i].size() <= i){
      cam_images_[i].resize(i + 1);
    }

    cam_images_[i].push_back(image);
    int s=0;
    for(int j=0;j<image.Points2D_board().size();j++)
    {
      if(image.Points2D_board()[j].size()!=0)
      {
        s++;
      }
    }
    cam_detected_board[i]+=s;
    // throw std::runtime_error("Error occurred with value: " + std::to_string(cam_detected_board[i]));
    return cam_images_[i].size() - 1;
  }
  inline size_t Calibration::AddImage_cam_undistorted(const class Image& image,int i) {
    if (cam_images_undistorted_[i].size() <= i){
      cam_images_undistorted_[i].resize(i + 1);
    }

    cam_images_undistorted_[i].push_back(image);
    int s=0;
    for(int j=0;j<image.Points2D_board().size();j++)
    {
      if(image.Points2D_board()[j].size()!=0)
      {
        s++;
      }
    }
    cam_detected_board_undistorted[i]+=s;
    // throw std::runtime_error("Error occurred with value: " + std::to_string(cam_detected_board[i]));
    return cam_images_undistorted_[i].size() - 1;
  }






  inline Eigen::Vector3d& Calibration::Point3D(uint32_t point_id) {
    return points3D_.at(point_id);
  }
  inline const Eigen::Vector3d& Calibration::Point3D(uint32_t point_id) const {
    return points3D_.at(point_id);
  }

  inline uint32_t Calibration::AddPoint3D(const Eigen::Vector3d& xyz) {
    uint32_t id = number_of_points3D;
    points3D_[number_of_points3D] = xyz;
    number_of_points3D++;
    return id;
  }

  inline std::map<uint32_t, Eigen::Vector3d>& Calibration::Points3D() {
    return points3D_;
  }
  inline const std::map<uint32_t, Eigen::Vector3d>& Calibration::Points3D() const {
    return points3D_;
  }

  inline void Calibration::SetPoints3D(const std::map<uint32_t, Eigen::Vector3d>& points3D) {
    points3D_ = points3D;
  }

  inline void Calibration::SetPoints3D_withboard(const std::map<uint32_t,std::map<uint32_t, Eigen::Vector3d>>& points3D_map) {
    points3D_map_= points3D_map;
  }


  inline std::vector<class Image>& Calibration::Images() {
    return images_;
  }
  inline const std::vector<class Image>& Calibration::Images() const {
    return images_;
  }
  inline std::vector<class Image>& Calibration::Cam_Images(int i) {
    return cam_images_.at(i);
  }
  inline const std::vector<class Image>& Calibration::Cam_Images(int i) const {
    return cam_images_.at(i);
  }
  inline std::vector<class Image>& Calibration::Cam_Images_Undistorted(int i) {
    return cam_images_undistorted_.at(i);
  }
  inline const std::vector<class Image>& Calibration::Cam_Images_Undistorted(int i) const {
    return cam_images_undistorted_.at(i);
  }

  inline double Calibration::CalibrationRms() const {
    return calibration_rms_;
  }

  inline void Calibration::SetCalibrationRms(double rms) {
    calibration_rms_ = rms;
  }


  inline double Calibration::Projectionerr_before() const {
    return projectionerr_before_;
  }

  inline void Calibration::Setprojectionerr_before(double error) {
    projectionerr_before_ = error;
  }
  
  inline double Calibration::Projectionerr_ba() const {
    return projectionerr_ba_;
  }

  inline void Calibration::Setprojectionerr_ba(double error) {
    projectionerr_ba_ = error;
  }



  inline std::vector<double>& Calibration::IntrinsicsStdDeviations() {
    return intrinsics_std_deviations_;
  }
  inline const std::vector<double>& Calibration::IntrinsicsStdDeviations() const {
    return intrinsics_std_deviations_;
  }

  inline void Calibration::SetIntrinsicsStdDeviations(const std::vector<double>& intrinsincs_rms) {
    intrinsics_std_deviations_ = intrinsincs_rms;
  }

  inline std::vector<double>& Calibration::HousingParamsStdDeviations() {
    return housing_params_std_deviations_;
  }

  inline const std::vector<double>& Calibration::HousingParamsStdDeviations() const {
    return housing_params_std_deviations_;
  }

  inline void Calibration::SetHousingParamsStdDeviations(const std::vector<double>& housing_std) {
    housing_params_std_deviations_ = housing_std;
  }

  inline std::vector<double>& Calibration::PerViewRms() {
    return per_view_rms_;
  }

  inline const std::vector<double>& Calibration::PerViewRms() const {
    return per_view_rms_;
  }

  inline void Calibration::SetPerViewRms(const std::vector<double>& per_view_rms) {
    per_view_rms_ = per_view_rms;
  }
  inline void Calibration::SetInitalPose(const std::vector<cv::Mat>&Cam_rotations,const std::vector<cv::Mat>&Cam_translations) {
    for(int i=0;i<Cam_rotations.size();i++)
    {
      cv::Mat R= Cam_rotations[i];
      cv::Mat t= Cam_translations[i];
      cv::Mat P = cv::Mat::eye(4, 4, CV_64F);
      R.copyTo(P(cv::Rect(0, 0, 3, 3)));  // 将旋转矩阵填入 P
      t.copyTo(P(cv::Rect(3, 0, 1, 3))); // 将平移向量填入 P
      initalpose[i]=P;
    }
  }
  inline  const cv::Mat & Calibration::InitalPose(int i) const{
    return initalpose[i];
  }
  inline  cv::Mat & Calibration::InitalPose(int i) {
    return initalpose[i];
  }

  inline void Calibration::SetbaPose(const std::vector<cv::Mat>&Cam_rotations_afterba,const std::vector<cv::Mat>&Cam_translations_afterba) {
    for(int i=0;i<Cam_rotations_afterba.size();i++)
    {
      cv::Mat R= Cam_rotations_afterba[i];
      cv::Mat t= Cam_translations_afterba[i];
      cv::Mat P = cv::Mat::eye(4, 4, CV_64F);
      R.copyTo(P(cv::Rect(0, 0, 3, 3)));  // 将旋转矩阵填入 P
      t.copyTo(P(cv::Rect(3, 0, 1, 3))); // 将平移向量填入 P
      bapose[i]=P;
    }
  }
  inline  const cv::Mat & Calibration::BaPose(int i) const{
    return bapose[i];
  }
  inline  cv::Mat & Calibration::BaPose(int i) {
    return bapose[i];
  }



  inline std::vector<int>& Calibration::PerView3DPointCount() {
    return per_view_3d_point_count_;
  }

  inline const std::vector<int>& Calibration::PerView3DPointCount() const {
    return per_view_3d_point_count_;
  }

  inline void Calibration::SetPerView3DPoints(const std::vector<int>& per_view_3d_point_count) {
    per_view_3d_point_count_ = per_view_3d_point_count;
  }

  inline void Calibration::SetCalibrationTargetInfo(const std::string& info) {
    calibration_target_info_ = info;
  }

  inline const std::string& Calibration::GetCalibrationTargetInfo() const {
    return calibration_target_info_;
  }

  inline const std::optional<colmap::Rigid3d>& Calibration::CameraToWorldStereo() const {
    return stereo_pose_;
  }

  inline void Calibration::SetCameraToWorldStereo(const std::optional<colmap::Rigid3d>& pose) {
    stereo_pose_ = pose;
  }
}