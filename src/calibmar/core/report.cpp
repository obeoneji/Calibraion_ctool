#include "report.h"

#include "calibmar/core/calibration_targets.h"

#include "colmap/util/misc.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>

namespace {

  void FormatMatrixToHtml(std::ostream& stream, const cv::Mat& matrix, const std::string& title) {
      stream <<  title   <<std::endl<<'[';
      for (int i = 0; i < matrix.rows; ++i) {
          stream <<'[';
          for (int j = 0; j < matrix.cols; ++j) {              
              if (matrix.type() == CV_32F) {
                  stream << matrix.at<float>(i, j);
              } 
              else 
              {
                  stream << matrix.at<double>(i, j);          
              }
              if(j<matrix.cols-1)
              {
                stream <<",";
              }
          }
          if(i<matrix.rows-1)
          {
            stream <<"]," <<std::endl;
          }
      }
      stream  <<']'<< std::endl;
  }
  std::vector<std::string> Split(const std::string& input, const std::string& regex) {
    // passing -1 as the submatch index parameter performs splitting
    std::regex re(regex);
    std::sregex_token_iterator first{input.begin(), input.end(), re, -1}, last;
    return {first, last};
  }

  std::string OriginToString(calibmar::ArucoGridOrigin origin) {
    switch (origin) {
      case calibmar::ArucoGridOrigin::TopLeft:
        return "tl";
      case calibmar::ArucoGridOrigin::TopRight:
        return "tr";
      case calibmar::ArucoGridOrigin::BottomLeft:
        return "bl";
      case calibmar::ArucoGridOrigin::BottomRight:
        return "br";
      default:
        return "unkown";
    }
  }

  std::string DirectionToString(calibmar::ArucoGridDirection direction) {
    switch (direction) {
      case calibmar::ArucoGridDirection::Horizontal:
        return "hor";
      case calibmar::ArucoGridDirection::Vertical:
        return "ver";
      default:
        return "unkown";
    }
  }

  calibmar::ArucoGridOrigin StringToOrigin(const std::string& origin) {
    calibmar::ArucoGridOrigin res;
    if (origin == "tl") {
      res = calibmar::ArucoGridOrigin::TopLeft;
    }
    else if (origin == "tr") {
      res = calibmar::ArucoGridOrigin::TopRight;
    }
    else if (origin == "bl") {
      res = calibmar::ArucoGridOrigin::BottomLeft;
    }
    else if (origin == "br") {
      res = calibmar::ArucoGridOrigin::BottomRight;
    }
    return res;
  }

  calibmar::ArucoGridDirection StringToDirection(const std::string& direction) {
    calibmar::ArucoGridDirection dir;
    if (direction == "hor") {
      dir = calibmar::ArucoGridDirection::Horizontal;
    }
    else if (direction == "ver") {
      dir = calibmar::ArucoGridDirection::Vertical;
    }
    return dir;
  }
}

namespace calibmar {
  namespace report {
    // void WriteCalibrationReport(const std::string& path, const Calibration& calibration) {
    //   std::ofstream out_file(path);
    //   GenerateResultString(out_file, calibration);
    //   out_file.close();
    // }
    void WriteCalibrationReport(const std::string& path, const std::vector<std::unique_ptr<Calibration>>& calibrations) {//liheng3
      std::ofstream out_file(path);
      GenerateResultString(out_file, calibrations);
      out_file.close();
    }
    void WriteCalibrationReport(const std::string& path, const Calibration& calibration) {//liheng3
      std::ofstream out_file(path);
      GenerateResultString(out_file, calibration);
      out_file.close();
    }

    // void WriteCalibrationYaml(const std::string& path, const Calibration& calibration) {
    void WriteCalibrationYaml(const std::string& path, const std::vector<std::unique_ptr<Calibration>>& calibrations) {//liheng3
      std::ofstream out_file(path);
      out_file.precision(10);
      GenerateCalibrationYaml(out_file, calibrations);
      out_file.close();
    }
    void WriteCalibrationYaml(const std::string& path, const Calibration& calibration) {//liheng3
      std::ofstream out_file(path);
      out_file.precision(10);
      GenerateCalibrationYaml(out_file, calibration);
      out_file.close();
    }
    void GenerateCalibrationYaml(std::ostream& stream, const Calibration& calibration) {//liheng3
      // const colmap::Camera& camera = calibration.Camera();
      const colmap::Camera& camera = calibration.Camera(0);
      // model
      stream << "model: " << camera.ModelName() << std::endl;
      stream << "# " << camera.ParamsInfo() << std::endl;
      std::string solvername;
      switch (calibration.G2O_solver_flag()) {
        case 0: // Eigen
          // 执行与 Eigen 求解器相关的操作  
          solvername = "Eigen";
          break;
        case 1: // Cholmod
          solvername = "Cholmod";
            // 执行与 Cholmod 求解器相关的操作
          break;
        case 2:
          // 处理未知索引的情况
          solvername = "Dense";
          break;
        }
      stream << "G2O Solver: " << solvername << std::endl;
      stream << "camnum: "<< calibration.cam_num << std::endl;
      stream << "Reprojection error before BA: "<< calibration.Projectionerr_before() << std::endl;
      stream << "Reprojection error after BA: "<< calibration.Projectionerr_ba() << std::endl;
      // parameters
      for(int i=0;i<calibration.cam_num;i++)
      {
        const colmap::Camera& camera = calibration.Camera(i); 
        const colmap::Camera& camera_undistorted = calibration.Camera_undistorted(i); 
        stream << "#CAM "<< i<<""<<std::endl;
        stream << "parameters_before_undistortion: [";  
        if (camera.params.size() > 0) {
          const std::vector<double>& params = camera.params;
          std::copy(params.begin(), params.end() - 1, std::ostream_iterator<double>(stream, ", "));
          stream << *(params.end() - 1);
        }
        stream << "]" << std::endl;
        if(calibration.Recomputeflag())
        {
          stream << "parameters_after_undistortion: [";  
          if (camera.params.size() > 0) {
            const std::vector<double>& params = camera_undistorted.params;
            std::copy(params.begin(), params.end() - 1, std::ostream_iterator<double>(stream, ", "));
            stream << *(params.end() - 1);
          }
          stream << "]" << std::endl;          
        }
        if(calibration.G2O_flag())
        {        
          const std::string  cam_string_before="Before bundle adjustment";
          FormatMatrixToHtml(stream, calibration.InitalPose(i),cam_string_before);
          const std::string  cam_string_after="After bundle adjustment";
          FormatMatrixToHtml(stream, calibration.BaPose(i),cam_string_after);          
        }
        else{
          const std::string  cam_string_before="Camera Pose ";
          FormatMatrixToHtml(stream, calibration.InitalPose(i),cam_string_before);
        }
      }

      // width & height
      stream << "width: " << camera.width << std::endl;
      stream << "height: " << camera.height << std::endl;
      // target
      stream << "# target: " << calibration.GetCalibrationTargetInfo();



    }
    // void GenerateCalibrationYaml(std::ostream& stream, const Calibration& calibration) {
    void GenerateCalibrationYaml(std::ostream& stream, const std::vector<std::unique_ptr<Calibration>>& calibrations) {//liheng3
      // const colmap::Camera& camera = calibration.Camera();
      const colmap::Camera& camera = calibrations[0]->Camera();
      const Calibration& calibration =*calibrations[0];
      // model
      stream << "model: " << camera.ModelName() << std::endl;
      stream << "camnum: "<< calibration.cam_num << std::endl;
      stream << "# " << camera.ParamsInfo() << std::endl;
      // parameters
      for (size_t i = 0; i < calibrations.size(); i++) {
        const Calibration& calibration =*calibrations[i];
        const colmap::Camera& camera = calibration.Camera();
        stream << "parameters: [";
        if (camera.params.size() > 0) {
          const std::vector<double>& params = camera.params;
          std::copy(params.begin(), params.end() - 1, std::ostream_iterator<double>(stream, ", "));
          stream << *(params.end() - 1);
        }

        stream << "]" << std::endl;
      }
      // std dev parameters
      const std::vector<double>& intrinsics_std_dev = calibrations[0]->IntrinsicsStdDeviations();
      if (intrinsics_std_dev.size() > 0) {
        stream << "# params_est_std_dev: ";
        std::copy(intrinsics_std_dev.begin(), intrinsics_std_dev.end() - 1, std::ostream_iterator<double>(stream, ", "));
        stream << *(intrinsics_std_dev.end() - 1) << std::endl;
      }
      // non_svp
      std::string non_svp_model_name = camera.IsCameraRefractive() ? camera.RefracModelName() : "NONE";
      stream << "non_svp_model: " << non_svp_model_name << std::endl;
      if (camera.IsCameraRefractive()) {
        // one of the info strings contains a new line...
        std::string params_info = std::regex_replace(camera.RefracParamsInfo(), std::regex("\\n"), " ");
        stream << "# " << params_info << std::endl;
      }

      for (size_t i = 0; i < calibrations.size(); i++) {
        const Calibration& calibration =*calibrations[i];
        const colmap::Camera& camera = calibration.Camera();
        stream << "non_svp_parameters: [";
        if (camera.IsCameraRefractive()) {
          const std::vector<double>& non_svp_params = camera.refrac_params;
          std::copy(non_svp_params.begin(), non_svp_params.end() - 1, std::ostream_iterator<double>(stream, ", "));
          stream << *(non_svp_params.end() - 1);
        }
        stream << "]" << std::endl;
        // std dev parameters non svp
        const std::vector<double>& housing_std_dev = calibration.HousingParamsStdDeviations();
        if (housing_std_dev.size() > 0) {
          stream << "# non_svp_est_std_dev: ";
          std::copy(housing_std_dev.begin(), housing_std_dev.end() - 1, std::ostream_iterator<double>(stream, ", "));
          stream << *(housing_std_dev.end() - 1) << std::endl;
        }
        // overall RMS
        stream << "# overall_rms: " << calibration.CalibrationRms() << std::endl;
      }


      // width & height
      stream << "width: " << camera.width << std::endl;
      stream << "height: " << camera.height << std::endl;
      // target
      // const Calibration& calibration =*calibrations[0];
      stream << "# target: " << calibration.GetCalibrationTargetInfo();

      // stereo
      // if (calibration.CameraToWorldStereo().has_value()) {
      //   Eigen::IOFormat yaml_format(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
      //   const colmap::Rigid3d& pose = calibration.CameraToWorldStereo().value();
      //   stream << "\n\n# stereo pose (camera to world, camera 3D point X_C can be transformed to 3D world point X_W by X_W = "
      //             "cam_to_world_rotation * X_C + cam_to_world_translation)";
      //   stream << "\ncam_to_world_rotation_rowmajor: " << pose.rotation.normalized().toRotationMatrix().format(yaml_format);
      //   stream << "\ncam_to_world_translation: " << pose.translation.format(yaml_format);
      //   stream << "\n# (distance to origin: " << pose.translation.norm() << ")";
      // }
    }

    // std::string GenerateResultString(const Calibration& calibration) {
    std::string GenerateResultString(const std::vector<std::unique_ptr<Calibration>>& calibrations) {//liheng3
      std::stringstream result_stream;
      GenerateResultString(result_stream, calibrations);
      return result_stream.str();
    }
    void GenerateResultString(std::ostream& stream, const Calibration& calibration)
    {//liheng3
      int first_column_width = 22;
      int field_width = 13;
      int cam_num=calibration.cam_num;
      const colmap::Camera& camera = calibration.Camera(0);
      // camera model
      std::string header = camera.IsCameraRefractive() ? "Camera & Housing Model:" : "Camera Model:";
      stream << header << std::endl << camera.ModelName();
      if (camera.IsCameraRefractive()) {
        stream << " " << camera.RefracModelName();
      }
      stream << std::endl << std::endl;
      //cam_num
      stream << "camnum: "<< std::endl<<calibration.cam_num;
      stream << std::endl << std::endl;
      //G2O_solver
      std::string solvername;
      switch (calibration.G2O_solver_flag()) {
        case 0: // Eigen
          // 执行与 Eigen 求解器相关的操作  
          solvername = "Eigen";
          break;
        case 1: // Cholmod
          solvername = "Cholmod";
            // 执行与 Cholmod 求解器相关的操作
          break;
        case 2:
          // 处理未知索引的情况
          solvername = "Dense";
          break;
        }
      stream << "G2O Solver:" <<std::endl<< solvername;
      stream << std::endl << std::endl; 
      // width & height
      stream << "Width & Height:" << std::endl;
      stream << camera.width << " " << camera.height;
      stream << std::endl << std::endl;
      stream << "Reprojection error before BA: "<< calibration.Projectionerr_before() << std::endl;
      stream << "Reprojection error after BA: "<< calibration.Projectionerr_ba() << std::endl;
      // parameter values
      for(int i=0;i<calibration.cam_num;i++)
      {
        stream << "camera "<<i<<":"<< std::endl;
        const colmap::Camera& camera_original = calibration.Camera(i); 
        const colmap::Camera& camera_undistorted = calibration.Camera_undistorted(i); 
        const std::vector<double>& params = camera_original.params;
        // parameter lables
        std::vector<std::string> param_names = Split(camera_original.ParamsInfo(), ", ");
        stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Parameters:";
        for (auto& value : param_names) {
          stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
        }
        stream << std::endl;        
        stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Values:";
        for (auto& value : params) {
          stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
        }
        stream << std::endl;
        stream << "camera_undistorted "<<i<<":"<< std::endl;
        const std::vector<double>& params_undis = camera_undistorted.params;
        // parameter lables
        std::vector<std::string> param_names_undis = Split(camera_undistorted.ParamsInfo(), ", ");
        stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Parameters:";
        for (auto& value : param_names_undis) {
          stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
        }
        stream << std::endl;        
        stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Values:";
        for (auto& value : params_undis) {
          stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
        }
        stream << std::endl;
        if(calibration.G2O_flag())
        {        
          const std::string  cam_string_before="Before bundle adjustment";
          FormatMatrixToHtml(stream, calibration.InitalPose(i),cam_string_before);
          const std::string  cam_string_after="After bundle adjustment";
          FormatMatrixToHtml(stream, calibration.BaPose(i),cam_string_after);          
        }
        else{
          const std::string  cam_string_before="Camera Pose ";
          FormatMatrixToHtml(stream, calibration.InitalPose(i),cam_string_before);
        }
      }

    // 对每个 calibration 执行操作
    }
    // void GenerateResultString(std::ostream& stream, const Calibration& calibration) {
    void GenerateResultString(std::ostream& stream, const std::vector<std::unique_ptr<Calibration>>& calibrations)
    {//liheng3
      int first_column_width = 22;
      int field_width = 13;
      int cam_num=calibrations.size();
      // const colmap::Camera& camera = calibration.Camera();
      const colmap::Camera& camera = calibrations[0]->Camera();
      // camera model
      std::string header = camera.IsCameraRefractive() ? "Camera & Housing Model:" : "Camera Model:";
      stream << header << std::endl << camera.ModelName();
      if (camera.IsCameraRefractive()) {
        stream << " " << camera.RefracModelName();
      }
      stream << std::endl << std::endl;
      //cam_num
      const Calibration& calibration = *calibrations[0];
      stream << "camnum: "<< calibration.cam_num << std::endl;
      // width & height
      stream << "Width & Height:" << std::endl;
      stream << camera.width << " " << camera.height;
      stream << std::endl << std::endl;
      // camera matrix
      stream << "Camera Matrix:" << std::endl << camera.CalibrationMatrix() << std::endl << std::endl;

      // parameter values
      for (size_t i = 0; i < calibrations.size(); i++) {
        stream << "camera "<<i<<":"<< std::endl;
        const Calibration& calibration = *calibrations[i];
        const colmap::Camera& camera = calibration.Camera();
        const std::vector<double>& params = camera.params;
        // parameter lables
        std::vector<std::string> param_names = Split(camera.ParamsInfo(), ", ");
        stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Parameters:";
        for (auto& value : param_names) {
          stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
        }
        stream << std::endl;        
        stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Values:";
        for (auto& value : params) {
          stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
        }
        stream << std::endl;
        // parameter std dev
        const std::vector<double>& intrinsics_std_dev = calibration.IntrinsicsStdDeviations();
        if (intrinsics_std_dev.size() > 0) {
          stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Est. Std. Deviations:";
          for (auto& value : intrinsics_std_dev) {
            stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
          }
          stream << std::endl;
        }
        if (camera.IsCameraRefractive()) {
          stream << std::endl;
          // housing labels
          std::vector<std::string> housing_param_names = Split(camera.RefracParamsInfo(), ", ");
          stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Housing Parameters:";
          for (auto& value : housing_param_names) {
            stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
          }
          stream << std::endl;
          // housing values
          const std::vector<double>& housing_params = camera.refrac_params;
          stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Values:";
          for (auto& value : housing_params) {
            stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
          }
          stream << std::endl;
          // housing std dev
          const std::vector<double>& housing_std_dev = calibration.HousingParamsStdDeviations();
          if (housing_std_dev.size() > 0) {
            stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Est. Std. Deviations:";
            for (auto& value : housing_std_dev) {
              stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
            }
            // stream << std::endl;
          }
        }
                // overall rms
        stream << std::endl << "Overall RMS: " << calibration.CalibrationRms() << std::endl;
        stream << std::endl;
      
    // 对每个 calibration 执行操作
      }

      // const std::vector<double>& params = camera.params;
      // stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Values:";
      // for (auto& value : params) {
      //   stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
      // }
      // stream << std::endl;
      // // parameter std dev
      // const std::vector<double>& intrinsics_std_dev = calibration.IntrinsicsStdDeviations();
      // if (intrinsics_std_dev.size() > 0) {
      //   stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Est. Std. Deviations:";
      //   for (auto& value : intrinsics_std_dev) {
      //     stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
      //   }
      //   stream << std::endl;
      // }
      // if (camera.IsCameraRefractive()) {
      //   stream << std::endl;
      //   // housing labels
      //   std::vector<std::string> housing_param_names = Split(camera.RefracParamsInfo(), ", ");
      //   stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Housing Parameters:";
      //   for (auto& value : housing_param_names) {
      //     stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
      //   }
      //   stream << std::endl;
      //   // housing values
      //   const std::vector<double>& housing_params = camera.refrac_params;
      //   stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Values:";
      //   for (auto& value : housing_params) {
      //     stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
      //   }
      //   stream << std::endl;
      //   // housing std dev
      //   const std::vector<double>& housing_std_dev = calibration.HousingParamsStdDeviations();
      //   if (housing_std_dev.size() > 0) {
      //     stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Est. Std. Deviations:";
      //     for (auto& value : housing_std_dev) {
      //       stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
      //     }
      //     stream << std::endl;
      //   }
      // }
      // optional stereo pose
      // if (calibration.CameraToWorldStereo().has_value()) {
      //   const colmap::Rigid3d& cam_to_world = calibration.CameraToWorldStereo().value();
      //   stream << std::endl << "Stereo Pose (camera to world R|t):" << std::endl;
      //   stream << cam_to_world.ToMatrix() << std::endl;
      //   stream << "(Distance to origin: " << cam_to_world.translation.norm() << ")" << std::endl;
      // }
      // // overall rms
      // stream << std::endl << "Overall RMS: " << calibration.CalibrationRms();
    }

    std::string GenerateCalibrationTargetInfo(const ChessboardFeatureExtractor::Options& options) {
      return "chessboard, " + std::to_string(options.chessboard_columns) + ", " + std::to_string(options.chessboard_rows) + ", " +
             std::to_string(options.square_size);
    }

    std::string GenerateCalibrationTargetInfo(const ArucoBoardFeatureExtractor::Options& options) {
      return "aruco grid board, " + calibration_targets::NameFromArucoType(options.aruco_type) + ", " +
             std::to_string(options.marker_cols) + ", " + std::to_string(options.marker_rows) + ", " +
             std::to_string(options.marker_size) + ", " + std::to_string(options.marker_spacing) + ", " +
             std::to_string(options.border_bits) + ", " + OriginToString(options.grid_origin) + ", " +
             DirectionToString(options.grid_direction);
    }
    std::string GenerateCalibrationTargetInfo(const CharucoBoardFeatureExtractor::Options& options) {
      return "charuco board, " + calibration_targets::NameFromArucoType(options.aruco_type) + ", " +
             std::to_string(options.columns) + ", " + std::to_string(options.rows) + ", " +
             std::to_string(options.marker_size) + ", " + std::to_string(options.square_size)+ ", " + std::to_string(options.marker_num)+ ", " + std::to_string(options.board_index)+ ", " + std::to_string(options.frame_num);
    }
    std::string GenerateCalibrationTargetInfo(const MultiCharucoBoardFeatureExtractor::Options& options) {
      return "charuco board, " + calibration_targets::NameFromArucoType(options.aruco_type) + ", " +
             std::to_string(options.columns) + ", " + std::to_string(options.rows) + ", " +
             std::to_string(options.marker_size) + ", " + std::to_string(options.square_size)+ ", " + std::to_string(options.marker_num)+ ", " + std::to_string(options.board_index);
    }

    std::string GenerateCalibrationTargetInfo(
        const std::variant<ArucoSiftFeatureExtractor::Options, SiftFeatureExtractor::Options>& options) {
      const ArucoSiftFeatureExtractor::Options* aruco_options = std::get_if<ArucoSiftFeatureExtractor::Options>(&options);
      if (aruco_options) {
        return "3D target, aruco: " + calibration_targets::NameFromArucoType(aruco_options->aruco_type) + ", " +
               std::to_string(aruco_options->masking_scale_factor);
      }
      else {
        return "3D target";
      }
    }
  }

  namespace {
    bool StartsWith(const std::string& string, const std::string& target) {
      return string.find(target) == 0;
    }

    inline void TrimEnd(std::string& s) {
      s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), s.end());
    }

    inline void TrimStart(std::string& s) {
      s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !std::isspace(ch); }));
    }

    inline void ToLower(std::string& s) {
      std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    }

    bool TryParse(double* d, const std::string& string, size_t* chars_used) {
      try {
        *d = std::stod(string, chars_used);
        return true;
      }
      catch (std::exception& e) {
        return false;
      }
    }

    bool TryParse(int* i, const std::string& string, size_t* chars_used) {
      try {
        *i = std::stoi(string, chars_used);
        return true;
      }
      catch (std::exception& e) {
        return false;
      }
    }

    template <typename T>
    bool TryParse(T* i, const std::string& string) {
      static_assert(sizeof(T) != sizeof(T), "TryParse must be specialized for this type!");
      return false;
    }

    template <>
    bool TryParse<int>(int* i, const std::string& string) {
      try {
        *i = std::stoi(string);
        return true;
      }
      catch (std::exception& e) {
        return false;
      }
    }

    template <>
    bool TryParse<double>(double* d, const std::string& string) {
      try {
        *d = std::stod(string);
        return true;
      }
      catch (std::exception& e) {
        return false;
      }
    }

    template <>
    bool TryParse<calibmar::ArucoMarkerTypes>(calibmar::ArucoMarkerTypes* type, const std::string& string) {
      for (const auto& name_type : calibration_targets::ArucoTypes()) {
        std::string aruco_name = name_type.first;
        std::string value_copy = string;
        ToLower(aruco_name);
        ToLower(value_copy);
        if (StartsWith(value_copy, aruco_name)) {
          *type = name_type.second;
          return true;
        }
      }
      return false;
    }

    template <>
    bool TryParse<calibmar::ArucoGridOrigin>(calibmar::ArucoGridOrigin* origin, const std::string& string) {
      *origin = StringToOrigin(string);
      return true;
    }

    template <>
    bool TryParse<calibmar::ArucoGridDirection>(calibmar::ArucoGridDirection* direction, const std::string& string) {
      *direction = StringToDirection(string);
      return true;
    }

    template <typename T>
    bool Parse(const std::vector<std::string>& values, size_t index, T* parameter) {
      return index < values.size() && TryParse(parameter, values[index]);
    }
  }

  ImportedParameters ImportedParameters::ImportFromYaml(const std::string& path,bool is_dome) {
    ImportedParameters parameters;

    std::filesystem::path file_path(path);
    if (!std::filesystem::exists(file_path) || !file_path.has_filename()) {
      return parameters;
    }

    std::ifstream input(path);

    parameters = ImportFromYaml(input, is_dome);
    parameters.directory = file_path.parent_path().string();

    return parameters;
  }

  ImportedParameters ImportedParameters::ImportFromYaml(std::istream& stream,bool is_dome) {
    ImportedParameters parameters;
    std::string line;
    bool model_param_check=false;
    bool model_check=false;
    bool first_house=true;
    while (std::getline(stream, line)) {
      TrimEnd(line);
      if (StartsWith(line, "model:")) {
        line.erase(0, sizeof("model: ") - 1);
        for (auto& [type, model] : CameraModel::CameraModels()) {
          if (model.model_name == line) {
            parameters.camera_model = type;
            break;
          }
        }
      }

      else if (StartsWith(line, "G2O Solver:")) {
        line.erase(0, sizeof("G2O Solver: ") - 1);
        if (line == "Eigen") {
            // 执行与 Eigen 求解器相关的操作 
            parameters.g2o_options=true;
            parameters.solver_index = 0;
        } else if (line == "Cholmod") {
            // 执行与 Cholmod 求解器相关的操作 
            parameters.g2o_options=true;
            parameters.solver_index = 1;
        } else if (line == "Dense") {
            // 执行与 Dense 求解器相关的操作  
            parameters.g2o_options=true;
            parameters.solver_index = 2;
        } else {
            // 处理未知的求解器类型
            parameters.g2o_options=false;
            std::cerr << "Unknown G2O Solver: " << line << std::endl;
        }

      }
      else if (StartsWith(line, "camnum: ")) {
        line.erase(0, sizeof("camnum: ") - 1);
        std::vector<std::string> values = colmap::CSVToVector<std::string>(line);
        Parse(values, 0, &parameters.camnum);
      }
      // else if (StartsWith(line, "parameters: [")) {
      //   line.erase(0, sizeof("parameters: [") - 1);
      //   double p;
      //   size_t chars_used;
      //   std::vector<double> current_parameters; //liheng3
      //   while (TryParse(&p, line, &chars_used)) {
      //     // parameters.camera_parameters.push_back(p);
      //     current_parameters.push_back(p);//liheng3
      //     // + the comma or closing bracket
      //     line.erase(0, chars_used + 1);
      //   }
      //   parameters.camera_parameters.push_back(current_parameters);//liheng3
      // }
      else if (StartsWith(line, "parameters: [") || StartsWith(line,"parameters_before_undistortion: [")) {
        std::string prefix = line.substr(0, line.find('[') + 1); // Get the prefix
        std::string parameters_line = line.substr(line.find('[') + 1);
        parameters_line.erase(parameters_line.end() - 1); // Remove the closing bracket

        double p;
        size_t chars_used;
        std::vector<double> current_parameters; // Store current parameters

        while (TryParse(&p, parameters_line, &chars_used)) {
            current_parameters.push_back(p);
            // Move to the next parameter, skipping the comma or space
            parameters_line.erase(0, chars_used + 1);
        }

        parameters.camera_parameters.push_back(current_parameters); // Add to the list of camera parameters
    }
      else if (StartsWith(line, "non_svp_model:")) {
        model_check=true;
        line.erase(0, sizeof("non_svp_model: ") - 1);
        for (auto& [type, model] : HousingInterface::HousingInterfaces()) {
          if (model.model_name == line) {
            parameters.housing_model = type;
            break;
          }
        }
      }
      else if (StartsWith(line, "non_svp_parameters: [")) {
        if(!first_house)
        {
          continue;
        }
        line.erase(0, sizeof("non_svp_parameters: [") - 1);
        double p;
        size_t chars_used;
        while (TryParse(&p, line, &chars_used)) {
          parameters.housing_parameters.push_back(p);
          // + the comma or closing bracket
          line.erase(0, chars_used + 1);
        }
        first_house=false;
        model_param_check=true;
      }
      else if (StartsWith(line, "# target: chessboard,")) {
        parameters.calibration_target = CalibrationTargetType::Chessboard;
        line.erase(0, sizeof("# target: chessboard,") - 1);

        std::vector<std::string> values = colmap::CSVToVector<std::string>(line);
        Parse(values, 0, &parameters.columns);
        Parse(values, 1, &parameters.rows);
        Parse(values, 2, &parameters.square_size);
      }
      else if (StartsWith(line, "# target: 3D target, aruco:")) {
        parameters.calibration_target = CalibrationTargetType::Target3DAruco;
        line.erase(0, sizeof("# target: 3D target, aruco:") - 1);

        std::vector<std::string> values = colmap::CSVToVector<std::string>(line);
        Parse(values, 0, &parameters.aruco_type);
        Parse(values, 1, &parameters.aruco_scale_factor);
      }
      else if (StartsWith(line, "# target: 3D target")) {
        parameters.calibration_target = CalibrationTargetType::Target3D;
      }
      else if (StartsWith(line, "# target: aruco grid board,")) {
        parameters.calibration_target = CalibrationTargetType::ArucoGridBoard;
        line.erase(0, sizeof("# target: aruco grid board,") - 1);

        std::vector<std::string> values = colmap::CSVToVector<std::string>(line);
        Parse(values, 0, &parameters.aruco_type);
        Parse(values, 1, &parameters.columns);
        Parse(values, 2, &parameters.rows);
        Parse(values, 3, &parameters.square_size);
        Parse(values, 4, &parameters.spacing);
        Parse(values, 5, &parameters.border_bits);
        Parse(values, 6, &parameters.grid_origin);
        Parse(values, 7, &parameters.grid_direction);
      }
      else if (StartsWith(line, "# target: charuco board,")) {
        parameters.calibration_target = CalibrationTargetType::ArucoGridBoard;
        line.erase(0, sizeof("# target: charuco board,") - 1);

        std::vector<std::string> values = colmap::CSVToVector<std::string>(line);
        Parse(values, 0, &parameters.aruco_type);
        Parse(values, 1, &parameters.columns);
        Parse(values, 2, &parameters.rows);
        Parse(values, 3, &parameters.marker_size);
        Parse(values, 4, &parameters.square_size);
        Parse(values, 5, &parameters.marker_num);
        Parse(values, 6, &parameters.board_index);
        Parse(values, 7, &parameters.frame_num);
      }      
    }
    if (is_dome && model_check==false && model_param_check==false) {
      for (auto& [type, model] : HousingInterface::HousingInterfaces()) {
        if (model.model_name == "DOMEPORT") {
          parameters.housing_model = type;
          break;
        }
      }
    
      double p;
      size_t chars_used;
      std::string house_params = "0.005,0,0,0.05,0.0001,1.0,1.473,1.334";

      while (TryParse(&p, house_params, &chars_used)) 
      {
        parameters.housing_parameters.push_back(p);
        // 移除解析后的数字和逗号
        house_params.erase(0, chars_used + 1);
      }
    }
    return parameters;
  }
}
