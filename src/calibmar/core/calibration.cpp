#include <calibmar/core/calibration.h>

#include "calibration.h"
#include <iostream>

namespace calibmar {
  void Calibration::GetCorrespondences(std::vector<std::vector<Eigen::Vector2d>>& points2D,
                                       std::vector<std::vector<Eigen::Vector3d>>& points3D) {
    points2D.clear();
    points3D.clear();
    points2D.resize(images_.size());
    points3D.resize(images_.size());
    for (size_t i = 0; i < images_.size(); i++) {
      class Image& image = images_[i];
      points2D[i].reserve(image.Correspondences().size());
      points3D[i].reserve(image.Correspondences().size());
      for (const auto& corresponcence : image.Correspondences()) 
      {
        points2D[i].push_back(image.Point2D(corresponcence.first));
        points3D[i].push_back(Point3D(corresponcence.second));
      }
    }
  }


  void Calibration::GetCorrespondences_board(std::vector<std::vector<Eigen::Vector2d>>& points2D,std::vector<std::vector<Eigen::Vector3d>>& points3D,int m) 
  {
    points2D.clear();
    points3D.clear();
    points2D.resize(cam_detected_board[m]);
    points3D.resize(cam_detected_board[m]);
    // throw std::runtime_error(std::to_string(cam_detected_board[m]));
    int index=0;
    std::vector<class Image>& images_tmp = cam_images_[m];
    for (size_t i = 0; i < images_tmp.size(); i++) 
    {
      class Image& image = images_tmp[i];
      for(size_t j=0;j<image.Correspondences_board().size();j++)
      { 
        if(image.Correspondences_board()[j].size()==0)
        {
          continue;
        }
        points2D[index].reserve(image.Correspondences_board()[j].size());
        points3D[index].reserve(image.Correspondences_board()[j].size());
        for (const auto& corresponcence : image.Correspondences_board()[j]) {
          points2D[index].push_back(image.Point2D_board(corresponcence.first,j));
          points3D[index].push_back(Point3D(corresponcence.second));
        }
        index++;
      }
    }
  }
  void Calibration::GetCorrespondences_board_undistorted(std::vector<std::vector<Eigen::Vector2d>>& points2D,std::vector<std::vector<Eigen::Vector3d>>& points3D,std::vector<int>&charucoidSets,std::vector<int>& boardindexSets,int m) 
  {
    points2D.clear();
    points3D.clear();
    charucoidSets.clear();
    boardindexSets.clear();
    points2D.resize(cam_detected_board_undistorted[m]);
    points3D.resize(cam_detected_board_undistorted[m]);
    // throw std::runtime_error(std::to_string(cam_detected_board[m]));
    int index=0;
    std::vector<class Image>& images_tmp = cam_images_undistorted_[m];
    for (size_t i = 0; i < images_tmp.size(); i++) 
    {
      class Image& image = images_tmp[i];
      boardindexSets.insert(boardindexSets.end(), image.Board().begin(), image.Board().end());
      for(size_t j=0;j<image.Correspondences_board().size();j++)
      { 
        if(image.Correspondences_board()[j].size()==0)
        {
          continue;
        }
        charucoidSets.insert(charucoidSets.end(), image.CharucoIds_board()[j].begin(), image.CharucoIds_board()[j].end());
        points2D[index].reserve(image.Correspondences_board()[j].size());
        points3D[index].reserve(image.Correspondences_board()[j].size());
        for (const auto& corresponcence : image.Correspondences_board()[j]) {
          points2D[index].push_back(image.Point2D_board(corresponcence.first,j));
          points3D[index].push_back(Point3D(corresponcence.second));
        }
        index++;
      }
    }
    // throw std::runtime_error(std::to_string(charucoidSets.size()));
  }

  void Calibration::InitializeFromReconstruction(const colmap::Reconstruction& reconstruction) {
    if (reconstruction.Cameras().size() != 1) {
      throw std::runtime_error("Reconstruction must contain exactly one camera!");
    }

    points3D_.clear();
    images_.clear();

    camera_ = reconstruction.Cameras().begin()->second;

    for (const auto image_id : reconstruction.RegImageIds()) {
      const colmap::Image& rec_image = reconstruction.Image(image_id);
      if (rec_image.CameraId() != camera_.camera_id) {
        continue;
      }

      calibmar::Image image;
      std::vector<Eigen::Vector2d> points2D;

      size_t i = 0;
      for (const auto& point2D : rec_image.Points2D()) {
        if (!point2D.HasPoint3D()) {
          continue;
        }

        // emplace will only add if key does not exist
        points3D_.emplace(point2D.point3D_id, reconstruction.Point3D(point2D.point3D_id).xyz);
        image.SetPoint3DforPoint2D(point2D.point3D_id, i);
        points2D.push_back(point2D.xy);
        i++;
      }

      image.SetPoints2D(points2D);
      image.SetPose(rec_image.CamFromWorld());
      images_.push_back(image);
    }
  }
}
