#include "calibmar/calibrators/basic_calibrator.h"
#include "calibmar/calibrators/general_calibration.h"
#include "calibmar/calibrators/opencv_calibration.h"
#include "calibmar/calibrators/g2o_calibration.h"
#include <algorithm>
#include <colmap/sensor/models.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <iostream>
#include <fstream>
namespace{
//   void throwErrorWithVector(const std::vector<bool>& vec) {
//     // 构建异常信息
//     std::ostringstream oss;
//     oss << "Error: Bool vector values are:\n";
    
//     for (bool value : vec) {
//         oss << (value ? "true" : "false") << " ";
//     }

//     // 抛出异常
//     throw std::runtime_error(oss.str());
// }
//   void throwErrorWithVector(const std::vector<int>& vec) {
//     // 构建异常信息
//     std::ostringstream oss;
//     oss << "Error: Vector values are: ";
//     for (int value : vec) {
//         oss << value << " ";
//     }

//     // 抛出异常
//     throw std::runtime_error(oss.str());
// }
  // void throwErrorWithVector(const std::vector<cv::Mat>& vec) {
  //   // 构建异常信息
  //   std::ostringstream oss;
  //   oss << "Error: Vector values are:\n";
    
  //   for (const cv::Mat& mat : vec) {
  //       if (mat.rows == 3 && mat.cols == 1) {
  //         oss << "[" << mat.at<double>(0, 0) << ", "
  //             << mat.at<double>(1, 0) << ", "
  //             << mat.at<double>(2, 0) << "] ";
  //       } 
  //       else if (mat.rows == 3 && mat.cols == 3) 
  //       {
  //         oss << "Matrix:\n";
  //         for (int i = 0; i < 3; ++i) 
  //         {
  //           for (int j = 0; j < 3; ++j) 
  //           {
  //             if (mat.type() == CV_32F) {
  //               oss << mat.at<float>(i, j) << (j < 2 ? ", " : ""); 
  //             }
  //             else{
  //               oss << mat.at<double>(i, j) << (j < 2 ? ", " : ""); 
  //             }
  //           }
  //           oss << "\n"; 
  //         }
  //       }
  //       else {
  //         oss << "[Invalid matrix size: " << mat.rows << "x" << mat.cols << "] ";
  //       }
  //   }

  //   // 抛出异常
  //   throw std::runtime_error(oss.str());
  // }
  // void throwErrorWithVector(const cv::Mat& vec) {
  //   // 构建异常信息
  //   std::ostringstream oss;
  //   oss << "Error: Vector values are:\n";
    
  //   cv::Mat mat=vec;
  //   if (mat.rows == 3 && mat.cols == 1) {
  //     oss << "[" << mat.at<double>(0, 0) << ", "
  //         << mat.at<double>(1, 0) << ", "
  //         << mat.at<double>(2, 0) << "] ";
  //   } 
  //   else if (mat.rows == 3 && mat.cols == 3) 
  //   {
  //     oss << "Matrix:\n";
  //     for (int i = 0; i < 3; ++i) 
  //     {
  //       for (int j = 0; j < 3; ++j) 
  //       {
  //         if (mat.type() == CV_32F) {
  //           oss << mat.at<float>(i, j) << (j < 2 ? ", " : ""); 
  //         }
  //         else{
  //           oss << mat.at<double>(i, j) << (j < 2 ? ", " : ""); 
  //         }
  //       }
  //       oss << "\n"; 
  //     }
  //   }
  //   else {
  //     oss << "[Invalid matrix size: " << mat.rows << "x" << mat.cols << "] ";
  //   }


  //   // 抛出异常
  //   throw std::runtime_error(oss.str());
  // }
  // void throwErrorWithVector(const std::vector<Eigen::Vector3d>& vec) {
  //     // 构建异常信息
  //     std::ostringstream oss;
  //     oss << "Error: Vector values are:\n";
      
  //     for (const Eigen::Vector3d& value : vec) {
  //       oss << "[" << value.x() << ", " << value.y() << ", " << value.z() << "] ";
  //     }
  //     // for (const Eigen::Vector2d& value : vec) {
  //     //     oss << "[ " << value.x() << " , " << value.y() << " ] ";
  //     // }

  //     // 抛出异常
  //     throw std::runtime_error(oss.str());
  // }
    void throwErrorWithVector(Eigen::Vector3d & vec) {
      // 构建异常信息
      std::ostringstream oss;
      oss << "Error: Vector values are:\n";
      
     
      oss << "[" << vec.x() << ", " << vec.y() << ", " << vec.z() << "] ";
    
      // for (const Eigen::Vector2d& value : vec) {
      //     oss << "[ " << value.x() << " , " << value.y() << " ] ";
      // }

      // 抛出异常
      throw std::runtime_error(oss.str());
  }
  void reprojection_selection(double & mean_error,std::vector<Eigen::Vector3d>& keypoint3d_selected,std::vector<Eigen::Vector2d>& keypoint2dj_selected,Eigen::Matrix3d& Kj,cv::Mat& R_saved,cv::Mat& T_saved,cv::Mat& dist) 
  {
    cv::Mat Kj_;
    eigen2cv(Kj, Kj_);
    // throwErrorWithVector(Kj_);
    cv::Mat keypoints3d_mat(keypoint3d_selected.size(), 1, CV_64FC3);
    cv::Mat keypoints2d_mat(keypoint3d_selected.size(), 1, CV_32FC2);
    std::vector<cv::Point2f> points2d_repro;
    for (size_t i = 0; i < keypoint3d_selected.size(); ++i) {
      keypoints3d_mat.at<cv::Vec3d>(i) = cv::Vec3d(keypoint3d_selected[i](0), keypoint3d_selected[i](1), keypoint3d_selected[i](2));
    }

    cv::projectPoints(keypoints3d_mat, R_saved, T_saved, Kj_, dist, keypoints2d_mat);
    cv::Mat kpts_repro(keypoint2dj_selected.size(), 1, CV_32FC2);
    for (size_t i = 0; i < keypoint2dj_selected.size(); ++i) 
    {
      kpts_repro.at<cv::Vec2f>(i) = cv::Vec2f(keypoint2dj_selected[i](0), keypoint2dj_selected[i](1));
    }
    // // 将 points2d_repro 转换为 cv::Mat
    // cv::Mat keypoints2d_mat(points2d_repro.size(), 1, CV_32FC2);
    // for (size_t i = 0; i < points2d_repro.size(); ++i) 
    // {
    //   keypoints2d_mat.at<cv::Vec2f>(i) = cv::Vec2f(points2d_repro[i].x, points2d_repro[i].y);
    // }
    keypoints2d_mat.convertTo(keypoints2d_mat, kpts_repro.type());
    cv::Mat diff = kpts_repro - keypoints2d_mat; // (N x 2)
    double norm_value = cv::norm(diff,cv::NORM_L2);
    // double norm_value=cv::norm(diff, norm, cv::NORM_L2); 
    mean_error = norm_value / kpts_repro.rows; 
    return ;
  }



  double Projection_err(std::vector<Eigen::Matrix3d> Cam_intrinsics,std::vector<cv::Mat> Cam_rotations,std::vector<cv::Mat> Cam_translations,std::vector<std::vector<int>>Cam_charucoidSets,std::vector<Eigen::Vector3d>point3d_dict,std::vector<int>point3d_dict_ids,std::vector<std::vector<Eigen::Vector2d>>&Cam_point2dSets,int &count_num)
 {
    double total_err = 0.0;

    for (int i = 0; i < point3d_dict_ids.size(); i++)
    {
      int point_id = point3d_dict_ids[i];
      Eigen::Vector3d point3d = point3d_dict[point_id];
      int num_=0;
      for (int j=0;j<Cam_charucoidSets.size();j++)
      {
          auto it = std::find(Cam_charucoidSets[j].begin(), Cam_charucoidSets[j].end(), point_id);
          if(it!=Cam_charucoidSets[j].end())
          {
              num_++;
          }
      }
      if(num_>1)
      {
        for (int j = 0; j < Cam_charucoidSets.size(); j++)
        {
          auto it = std::find(Cam_charucoidSets[j].begin(), Cam_charucoidSets[j].end(), point_id);
          if (it != Cam_charucoidSets[j].end())
          {
            int index = std::distance(Cam_charucoidSets[j].begin(), it);
            Eigen::Vector2d eigen_2d_point=Cam_point2dSets[j][index]; // 假设已经初始化


            Eigen::Matrix3d K = Cam_intrinsics[j];
            cv::Mat R = Cam_rotations[j];
            cv::Mat T = Cam_translations[j];
            Eigen::Matrix<double, 3, 3> R_eigen; // 旋转向量
            Eigen::Matrix<double, 3, 1> T_eigen; // 平移向量
            cv::cv2eigen(R, R_eigen);
            cv::cv2eigen(T, T_eigen);
            Eigen::Matrix<double, 4, 4> RT;
            RT.setZero(); 
            RT.block<3, 3>(0, 0) = R_eigen;
            RT.block<3, 1>(0, 3) = T_eigen;
            RT(3, 3) = 1.0; 

            Eigen::Vector4d point3d_h(point3d(0), point3d(1), point3d(2), 1.0);
            Eigen::Vector4d transformed_point = RT * point3d_h; 
            Eigen::Vector3d transformed_point_K; 
            transformed_point_K << transformed_point(0), transformed_point(1), transformed_point(2);
            Eigen::Matrix<double, 3, 1> projection = K * transformed_point_K;
            projection /= projection(2); // 归一化

            double dx = projection(0) - eigen_2d_point(0);
            double dy = projection(1) - eigen_2d_point(1);
            double err = sqrt(dx * dx + dy * dy);
            // throwErrorWithVector(projection);
            // throw std::runtime_error("point_id:"+std::to_string(point_id));
            // throw std::runtime_error("index:"+std::to_string(index)+','+std::to_string(projection(0))+','+std::to_string(projection(1))+','+std::to_string(eigen_2d_point(0))+','+std::to_string(eigen_2d_point(1)));
            total_err += err;
            count_num++;
          }
        }
      }
    }

    return (count_num > 0) ? (total_err / count_num) : 0.0; // 返回平均误差
  }
  std::vector<int> getIntersection(const std::vector<int>& vec1, const std::vector<int>& vec2)
  {
    std::unordered_set<int> set(vec1.begin(), vec1.end());
    std::vector<int> intersection;

    for (int num : vec2) {
        if (set.find(num) != set.end()) {
            intersection.push_back(num);
        }
    }
    // 对交集进行排序
    std::sort(intersection.begin(), intersection.end());
    return intersection;
  }
  bool get_points(int i,int j,std::vector<std::vector<int>>& Cam_charucoidSets,std::vector<std::vector<int>> Cam_boardindexSets,std::vector<std::vector<cv::Mat>> &Board_rotations ,std::vector<std::vector<cv::Mat>>&Board_translations,std::vector<int>& common_markers,
      std::vector<cv::Mat>& former_rotations,std::vector<cv::Mat>&latter_rotations,std::vector<cv::Mat>&former_translations,std::vector<cv::Mat>&latter_translations,
      std::vector<int>& common_ids,std::vector<int> &maskj)
  { 
    common_markers=getIntersection(Cam_boardindexSets[i],Cam_boardindexSets[j]);
    for (int intersection_board : common_markers) 
    {
      auto it1 = std::find(Cam_boardindexSets[i].begin(), Cam_boardindexSets[i].end(), intersection_board);
      auto it2 = std::find(Cam_boardindexSets[j].begin(), Cam_boardindexSets[j].end(), intersection_board);

      if (it1 != Cam_boardindexSets[i].end()) {
        // indices1.push_back(std::distance(vec1.begin(), it1));
        former_rotations.push_back(Board_rotations[i][std::distance(Cam_boardindexSets[i].begin(), it1)]);
        former_translations.push_back(Board_translations[i][std::distance(Cam_boardindexSets[i].begin(), it1)]);
      }
      if (it2 != Cam_boardindexSets[j].end()) {
        // indices2.push_back(std::distance(vec2.begin(), it2));
        latter_rotations.push_back(Board_rotations[j][std::distance(Cam_boardindexSets[j].begin(), it2)]);
        latter_translations.push_back(Board_translations[j][std::distance(Cam_boardindexSets[j].begin(), it2)]);
      }
    }
    common_ids=getIntersection(Cam_charucoidSets[i],Cam_charucoidSets[j]);
    for (int id : common_ids) 
    {
      auto it2 = std::find(Cam_charucoidSets[j].begin(), Cam_charucoidSets[j].end(), id);
      if (it2 != Cam_charucoidSets[j].end()) {
        // indices2.push_back(std::distance(vec2.begin(), it2));
        maskj.push_back(std::distance(Cam_charucoidSets[j].begin(), it2));
      }
    }


    if (common_markers.size()>0 && common_ids.size()>0)
    {
      return true;
    }
    else{
      return false;
    }


    // std::vector<int> common_markers;
    
  }

  void get_order_ij(int &former_index, int &latter_index,const std::vector<int>& saved, const std::vector<int>& not_saved,std::vector<std::vector<int>>& Cam_charucoidSets,
                    std::vector<std::vector<int>> &Cam_boardindexSets,std::vector<std::vector<cv::Mat>> &Board_rotations ,std::vector<std::vector<cv::Mat>>&Board_translations,
                    std::vector<int>& common_markers,std::vector<cv::Mat>& former_rotations,std::vector<cv::Mat>&latter_rotations,std::vector<cv::Mat>&former_translations,std::vector<cv::Mat>&latter_translations,
                    std::vector<int>& common_ids,std::vector<int> &maskj) 
  {
    for (int i = 0; i < saved.size(); i++) 
    {
      int closestIndex = -1;
      int form_index=saved[i];
      // 在 A 中查找与 B[i] 最近的元素
      for (int j = 0; j < not_saved.size(); j++) 
      {
        int lat_index=not_saved[j];
        bool ret = get_points(form_index,lat_index,Cam_charucoidSets,Cam_boardindexSets,Board_rotations,Board_translations,common_markers,former_rotations,latter_rotations,former_translations,latter_translations,common_ids,maskj);
        if (ret) 
        {
          former_index = form_index;
          latter_index = lat_index;
          return;
        }
        else
        {
          continue;
        }
      }
    }
    throw std::runtime_error("Can not find detected board in cam i and j.");
  }
  void add_3dpoints(int cam_index,std::vector<Eigen::Vector3d>& point3d_dict,std::vector<int>&point3d_dict_ids,std::vector<std::vector<Eigen::Vector2d>> Cam_point2dSets,std::vector<bool>& point3d_dict_mask, std::vector<std::vector<int>> Cam_boardindexSets,std::vector<std::vector<int>> Cam_charucoidSets,std::vector<cv::Mat>Cam_rotations,
  std::vector<cv::Mat> Cam_translations, std::vector<Eigen::Matrix3d>Cam_intrinsics, std::vector<std::vector<cv::Mat>> Board_rotations, std::vector<std::vector<cv::Mat>>Board_translations,int pattern)
  {
    std::vector<Eigen::Vector2d> KEYPOINTS2D_i=Cam_point2dSets[cam_index];
    std::vector<int> mask_i=Cam_charucoidSets[cam_index];
    cv::Mat R=Cam_rotations[cam_index];
    cv::Mat T=Cam_translations[cam_index];
    Eigen::Matrix<double, 3, 3> R_eigen;
    Eigen::Matrix<double, 3, 1> T_eigen;
    // cv::Mat R_;
    // cv::Rodrigues(R, R_);
    cv::cv2eigen(R, R_eigen);
    cv::cv2eigen(T, T_eigen);
    Eigen::Matrix3d Ki=Cam_intrinsics[cam_index];
    for(int i=0;i<Cam_boardindexSets[cam_index].size();i++)
    {
      int board_index = Cam_boardindexSets[cam_index][i];
      cv::Mat rvec_board = Board_rotations[cam_index][i];
      cv::Mat tvec_board = Board_translations[cam_index][i];
      cv::Mat rvec_board_;// 转换为 Eigen 矩阵
      Eigen::Matrix<double, 3, 3> rvec_eigen; // 旋转向量
      Eigen::Matrix<double, 3, 1> tvec_eigen; // 平移向量
      cv::Rodrigues(rvec_board, rvec_board_);

      cv::cv2eigen(rvec_board_, rvec_eigen);
      cv::cv2eigen(tvec_board, tvec_eigen);
      for (int m = board_index * pattern; m < (board_index + 1) * pattern;  m++)
      {            
        if (std::find(mask_i.begin(), mask_i.end(), m) != mask_i.end() && point3d_dict_mask[m]==false) 
        {
          auto p = std::find(mask_i.begin(), mask_i.end(), m);
          int index = std::distance(mask_i.begin(), p);
          double u = KEYPOINTS2D_i[index][0];
          double v = KEYPOINTS2D_i[index][1];
          Eigen::Vector3d  uvpoint(u, v, 1);
          Eigen::Vector3d leftSideMat = rvec_eigen.inverse() * Ki.inverse() * uvpoint;
          Eigen::Vector3d rightSideMat = rvec_eigen.inverse() * tvec_eigen;
          double s = (0 + rightSideMat(2)) / leftSideMat(2);
          Eigen::Vector3d X = rvec_eigen.inverse() * ((s * Ki.inverse() * uvpoint) - tvec_eigen);
          Eigen::Vector3d P_world = R_eigen.inverse() * (rvec_eigen * X + tvec_eigen - T_eigen);
          // throwErrorWithVector(P_world); 
          // throw std::runtime_error(std::to_string(m));
          Eigen::Vector3d P_world1(P_world.x(), P_world.y(), P_world.z());
          point3d_dict[m] = P_world1;
          point3d_dict_mask[m]=true;
          point3d_dict_ids.push_back(m);
        }
        else{
          continue;
        }
      }
    }  
  }

}

namespace calibmar 
{
  void BasicCalibrator::Options::Check() 
  {
    if (!use_intrinsics_guess && (image_size.first == 0 || image_size.second == 0)) 
    {
      throw std::runtime_error("Image size must be set!.");
    }
    if(root_cam>cam_num)
    {
      throw std::runtime_error("Root cam index > total number of camera system!."); 
    }

  }

  BasicCalibrator::BasicCalibrator(const Options& options) : options_(options) {}
  //calibration intrinsic
  void BasicCalibrator::Calibrate(Calibration& calibration) 
  {
    options_.Check();
    // throw std::runtime_error(std::to_string(options_.pattern));
    if(calibration.Recomputeflag())
    {
      //output
      std::vector<Eigen::Matrix3d> Cam_intrinsics(options_.cam_num);//N*3*3 
      std::vector<cv::Mat> Cam_rotations(options_.cam_num);//N*3*3
      std::vector<cv::Mat> Cam_translations(options_.cam_num);//N*3*1 

      std::vector<cv::Mat> Cam_rotations_afterba(options_.cam_num);//N*3*3
      std::vector<cv::Mat> Cam_translations_afterba(options_.cam_num);//N*3*1 
      for(int i=0;i<options_.cam_num;i++)
      {
        Cam_rotations[i]=cv::Mat::eye(3, 3, CV_64F); // 单位矩阵
        Cam_translations[i]= cv::Mat::zeros(3, 1, CV_64F);
        Cam_rotations_afterba[i]=cv::Mat::eye(3, 3, CV_64F); // 单位矩阵
        Cam_translations_afterba[i]= cv::Mat::zeros(3, 1, CV_64F);
      }
      // throwErrorWithVector(Cam_rotations[0]);
      //use
      std::vector<std::vector<int>> Cam_charucoidSets(options_.cam_num);//8*all_points*1
      std::vector<std::vector<Eigen::Vector2d>> Cam_point2dSets(options_.cam_num);//8*all_points_per_cam*1
      std::vector<std::vector<Eigen::Vector3d>> Cam_point3dSets(options_.cam_num);//8*all_points_per_cam*1

      std::vector<std::vector<int>> Cam_boardindexSets(options_.cam_num);//8*n*1
      std::vector<std::vector<cv::Mat>> Board_rotations(options_.cam_num);//8*n*(3*1)
      std::vector<std::vector<cv::Mat>> Board_translations(options_.cam_num); //8*n*(3*1)

      int allpoint_=options_.frame_num * options_.pattern * options_.board_num;
      std::vector<Eigen::Vector3d> point3d_dict;
      for(int i=0;i<allpoint_;i++)
      {
        point3d_dict.push_back(Eigen::Vector3d::Zero());
      }
      std::vector<bool> point3d_dict_mask(options_.frame_num * options_.pattern * options_.board_num);
      std::vector<int> point3d_dict_ids;

    //extraction
      for(int i=0;i<options_.cam_num;i++)
      {
        if (calibration.Cam_Images_Undistorted(i).size() == 0) {
          throw std::runtime_error("No images to calibrate from.");
        }
        if (calibration.Points3D().size() == 0) {
          throw std::runtime_error("3D Points not set.");
        }
        colmap::Camera& camera = calibration.Camera_undistorted(i);
        options_.camera_model = CameraModelType::OpenCVCameraModel;
        if (options_.use_intrinsics_guess && camera.model_id == colmap::CameraModelId::kInvalid) {
          throw std::runtime_error("Intrinsics guess specified, but camera not initialized.");
        }

        if (!options_.use_intrinsics_guess) {
          camera.width = options_.image_size.first;
          camera.height = options_.image_size.second;
          camera.model_id = colmap::CameraModelNameToId(calibmar::CameraModel::CameraModels().at(options_.camera_model).model_name);
        }

        std::vector<std::vector<Eigen::Vector2d>> pointSets2D;
        std::vector<std::vector<Eigen::Vector3d>> pointSets3D;
        std::vector<int> charucoidSets;
        std::vector<int> boardindexSets;
        calibration.GetCorrespondences_board_undistorted(pointSets2D, pointSets3D,charucoidSets,boardindexSets, i);
        if(pointSets3D.size()!=boardindexSets.size())
        {
          throw std::runtime_error(std::to_string(pointSets3D.size()) + " " + std::to_string(boardindexSets.size()));
        }

        std::vector<colmap::Rigid3d> poses;
        std::vector<Eigen::Vector2d> concatenated2dPoints;
        std::vector<Eigen::Vector3d> concatenated3dPoints;
        for (const auto& pointSet : pointSets2D) {
          concatenated2dPoints.insert(concatenated2dPoints.end(), pointSet.begin(), pointSet.end());
        }
        for (const auto& pointSet : pointSets3D) {
          concatenated3dPoints.insert(concatenated3dPoints.end(), pointSet.begin(), pointSet.end());
        }
        // std::vector<Eigen::Quaterniond*> rotations(pointSets3D.size());
        // std::vector<Eigen::Vector3d*> translations(pointSets3D.size());
        std::vector<cv::Mat> rotations;//N*3*1
        std::vector<cv::Mat> translations;//N*3*1
        cv::Mat intrinsic;//N*3*1 
        std::vector<double> std_deviations_intrinsics, std_deviations_extrinsics,per_view_rms;
        double rms = opencv_calibration::CalibrateCamera(pointSets3D, pointSets2D, camera, false, false,rotations, translations,std_deviations_intrinsics,std_deviations_extrinsics,per_view_rms);
        colmap::Camera& camera_undistorted = calibration.Camera_undistorted(i); 
        Eigen::Matrix3d K = camera_undistorted.CalibrationMatrix();
        // throw std::runtime_error(std::to_string(rotations[0].rows)+" "+std::to_string(rotations[0].cols));

        Cam_intrinsics[i]=K;
        Cam_boardindexSets[i]=boardindexSets;
        Board_rotations[i]=rotations;
        Board_translations[i]=translations;
        Cam_point2dSets[i]=concatenated2dPoints;
        Cam_point3dSets[i]=concatenated3dPoints;  
        Cam_charucoidSets[i]=charucoidSets;

        // throwErrorWithVector(rotations);

        calibration.SetCalibrationRms(rms);
        calibration.SetPerViewRms(per_view_rms);
        calibration.SetIntrinsicsStdDeviations(std_deviations_intrinsics);

      }
      // throwErrorWithVector(Cam_charucoidSets[1]);
      // throwErrorWithVector(Cam_boardindexSets[1]);
    //inital extrinsic
      if(options_.root_cam)
      {
        Cam_rotations[options_.root_cam-1] = cv::Mat::eye(3, 3, CV_64F); 
        Cam_translations[options_.root_cam-1]= cv::Mat::zeros(3, 1, CV_64F); 
      }    
      std::vector<int> saved; // 数组 saved
      std::vector<int> not_saved(options_.cam_num); // 数组 B 初始化为 {2}
      for (int i = 0; i < options_.cam_num; ++i) 
      {
        not_saved[i] = i; // 填充 0, 1, 2, ..., cam_num - 1
      }
      int indexToRemove=options_.root_cam-1;
      saved.push_back(indexToRemove);
      not_saved.erase(not_saved.begin() + indexToRemove);
      // throw std::runtime_error(std::to_string(not_saved[0])+" "+std::to_string(not_saved[1])+" "+std::to_string(not_saved[2])+" "+std::to_string(not_saved[3])+" "+std::to_string(not_saved[4]));
      for(int index=0;index<options_.cam_num;index++)
      {
        if(index==0)
        {
          ;
        }
        else
        {
          int former_index;
          int latter_index;
          std::vector<cv::Mat> former_rotations;
          std::vector<cv::Mat> latter_rotations;
          std::vector<cv::Mat> former_translations;
          std::vector<cv::Mat> latter_translations;
          std::vector<int> common_markers;
          std::vector<Eigen::Vector2d> keypoint2dj_selected;
          std::vector<int> maskj;
          std::vector<int> common_ids;
          get_order_ij(former_index,latter_index,saved,not_saved,Cam_charucoidSets,Cam_boardindexSets,Board_rotations,Board_translations,common_markers,
          former_rotations,latter_rotations,former_translations,latter_translations,common_ids,maskj);
          // throw std::runtime_error(std::to_string(latter_index));
          for(int p=0;p<maskj.size();p++)
          {
            int maskj_index=maskj[p];
            keypoint2dj_selected.push_back(Cam_point2dSets[latter_index][maskj_index]);
          }
          // throwErrorWithVector(keypoint2dj_selected);

          // std::vector<Eigen::Vector2d> keypoint2dj=Cam_point2dSets[j];
          
          auto it = std::find(not_saved.begin(), not_saved.end(), latter_index);
          int removeIndex = std::distance(not_saved.begin(), it);
          not_saved.erase(not_saved.begin() + removeIndex);
          saved.push_back(latter_index);
          Eigen::Matrix3d Ki=Cam_intrinsics[former_index];
          Eigen::Matrix3d Kj=Cam_intrinsics[latter_index];

          std::vector<Eigen::Vector3d> points_pro;
          for(int mask_index=0;mask_index<common_ids.size();mask_index++)
          {
            int real_mask_index=common_ids[mask_index];
            if(point3d_dict_mask[real_mask_index]==true)
            {
              points_pro.push_back(point3d_dict[real_mask_index]);
            }
          }
          // throwErrorWithVector(Cam_charucoidSets[2]);
          // throw std::runtime_error(std::to_string(points_pro.size())+std::to_string(common_ids.size()));
          double error=10000;
          cv::Mat R_saved;
          cv::Mat T_saved;
          for(int n=0;n<common_markers.size();n++)
          {
            int common_indice=common_markers[n];
            cv::Mat rvec1 = former_rotations[n];
            cv::Mat tvec1 = former_translations[n];
            cv::Mat rvec2 = latter_rotations[n];
            cv::Mat tvec2 = latter_translations[n];
            cv::Mat rvec1_;
            cv::Mat rvec2_;
            // throwErrorWithVector(rvec1);
            cv::Rodrigues(rvec1, rvec1_);
            cv::Mat P1 = cv::Mat::eye(4, 4, CV_64F);
            rvec1_.copyTo(P1(cv::Rect(0, 0, 3, 3)));  // 将旋转矩阵填入 P1
            tvec1.copyTo(P1(cv::Rect(3, 0, 1, 3))); // 将平移向量填入 P

            cv::Rodrigues(rvec2, rvec2_);
            cv::Mat P2 = cv::Mat::eye(4, 4, CV_64F);
            rvec2_.copyTo(P2(cv::Rect(0, 0, 3, 3)));  // 将旋转矩阵填入 P1
            tvec2.copyTo(P2(cv::Rect(3, 0, 1, 3))); // 将平移向量填入 P1
            cv::Mat T_21 = P2 * P1.inv(); // 计算 T_21

            cv::Mat R_relative = T_21(cv::Rect(0, 0, 3, 3)); 
            // throwErrorWithVector(R_relative);
            cv::Mat T_relative = T_21(cv::Rect(3, 0, 1, 3)); 
            // cv::Mat R_former=Cam_rotations[former_index];
            // throwErrorWithVector(R_former);
            cv::Mat R_j = R_relative * Cam_rotations[former_index]; 
            cv::Mat T_j = Cam_translations[former_index] + Cam_rotations[former_index] * T_relative; 
            // throwErrorWithVector(R_j);
            double mean_error;
            cv::Mat dist = cv::Mat::zeros(5, 1, CV_32F);
            reprojection_selection(mean_error,points_pro, keypoint2dj_selected, Kj, R_j, T_j, dist);
            // throw std::runtime_error(std::to_string(mean_error));
            if(mean_error<error)
            {
              error=mean_error;
              R_saved=R_j;
              T_saved=T_j;
            }
          }
          Cam_rotations[latter_index]=R_saved;
          Cam_translations[latter_index]=T_saved;
          // throwErrorWithVector(Cam_rotations);

          // throwErrorWithVector(points_pro);
          // throw std::runtime_error(std::to_string(points_pro.size())+std::to_string(common_ids.size()));
        }
        int cam_index=saved[index];
        add_3dpoints(cam_index,point3d_dict,point3d_dict_ids,Cam_point2dSets,point3d_dict_mask,Cam_boardindexSets,Cam_charucoidSets,Cam_rotations,Cam_translations,Cam_intrinsics,Board_rotations,Board_translations,options_.pattern);
        // throwErrorWithVector(point3d_dict);
      }
      std::sort(point3d_dict_ids.begin(), point3d_dict_ids.end());
      calibration.SetInitalPose(Cam_rotations,Cam_translations);
      int count_num=0;
      int count_num_ba=0;
      double projection_error=Projection_err(Cam_intrinsics,Cam_rotations,Cam_translations,Cam_charucoidSets,point3d_dict,point3d_dict_ids,Cam_point2dSets,count_num);
      // throw std::runtime_error(std::to_string(projection_error)+','+std::to_string(count_num));
      calibration.Setprojectionerr_before(projection_error);


      if(calibration.G2O_flag())
      {
        g2o_calibration::testG2o(calibration,options_.cam_num,options_.root_cam,Cam_point2dSets,Cam_point3dSets,Cam_charucoidSets,point3d_dict,point3d_dict_ids,Cam_rotations_afterba,Cam_translations_afterba,options_.solver_index);
        // throwErrorWithVector(Cam_translations_afterba);
        calibration.SetbaPose(Cam_rotations_afterba,Cam_translations_afterba);
      }
      double projection_error_ba=Projection_err(Cam_intrinsics,Cam_rotations_afterba,Cam_translations_afterba,Cam_charucoidSets,point3d_dict,point3d_dict_ids,Cam_point2dSets,count_num_ba);
      calibration.Setprojectionerr_ba(projection_error_ba);
      // throw std::runtime_error(std::to_string(projection_error_ba)+','+std::to_string(count_num_ba));
    }
    else
    {
      for(int i=0;i<options_.cam_num;i++)
      {
        if (calibration.Cam_Images(i).size() == 0) {
          throw std::runtime_error("No images to calibrate from.");
        }
        if (calibration.Points3D().size() == 0) {
          throw std::runtime_error("3D Points not set.");
        }
        colmap::Camera& camera = calibration.Camera(i);

        if (options_.use_intrinsics_guess && camera.model_id == colmap::CameraModelId::kInvalid) {
          throw std::runtime_error("Intrinsics guess specified, but camera not initialized.");
        }

        if (!options_.use_intrinsics_guess) {
          camera.width = options_.image_size.first;
          camera.height = options_.image_size.second;
          camera.model_id = colmap::CameraModelNameToId(calibmar::CameraModel::CameraModels().at(options_.camera_model).model_name);
        }

        std::vector<std::vector<Eigen::Vector2d>> pointSets2D;
        std::vector<std::vector<Eigen::Vector3d>> pointSets3D;
        calibration.GetCorrespondences_board(pointSets2D, pointSets3D, i);
        // throw std::runtime_error(std::to_string(pointSets2D.size()));
        std::vector<colmap::Rigid3d> poses;
        // std::vector<Eigen::Quaterniond*> rotations(pointSets3D.size());
        // std::vector<Eigen::Vector3d*> translations(pointSets3D.size());
        std::vector<cv::Mat> rotations;
        std::vector<cv::Mat> translations;
        std::vector<double> std_deviations_intrinsics, std_deviations_extrinsics,per_view_rms;
        double rms = opencv_calibration::CalibrateCamera(pointSets3D, pointSets2D, camera, false, false,rotations, translations,std_deviations_intrinsics,std_deviations_extrinsics,per_view_rms);



        // general_calibration::CalibrateCamera(pointSets3D, pointSets2D, camera, options_.use_intrinsics_guess, poses,
        //                                      &std_deviations_intrinsics);

        // double rms = general_calibration::CalculateOverallRMS(pointSets3D, pointSets2D, poses, camera, per_view_rms);

        calibration.SetCalibrationRms(rms);
        calibration.SetPerViewRms(per_view_rms);
        calibration.SetIntrinsicsStdDeviations(std_deviations_intrinsics);

      }
      // throw std::runtime_error("finished");
    }
  }
}