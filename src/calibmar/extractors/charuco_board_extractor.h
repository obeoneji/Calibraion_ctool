#pragma once

#include "calibmar/core/calibration_targets.h"
#include "extractor.h"
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/aruco/charuco.hpp>

namespace calibmar {
  class CharucoBoardFeatureExtractor : public FeatureExtractor {
    public:
      struct Options {
        ArucoMarkerTypes aruco_type = ArucoMarkerTypes::DICT_4X4_250;
        // number of chessboard rows
        int rows = 6;
        // number of chessboard columns
        int columns = 6;
        // board_idx
        int board_index=1;
        //the num of markers
        int marker_num=17;
        // size of the Charuco board square (in meters)
        double square_size = 0.04;
        // size of the Charuco board marker (in meters)
        double marker_size = 0.02;
        // frame_num
        int frame_num=5;
        // if disabled, corner detection will be iteratively attempted with multiple settings
      };

      CharucoBoardFeatureExtractor(const Options& options);

      Status Extract(Image& image, const Pixmap& pixmap) override;
      // Get target 3D points. Useful to set in the calibration.
      //
      // @return 3D points mapped to their index.
      const std::map<uint32_t, Eigen::Vector3d>& Points3D();
      const std::map<uint32_t,std::map<uint32_t, Eigen::Vector3d>>&Points3D_withboard();
    private:
      Options options_;
      std::map<uint32_t,std::map<uint32_t, Eigen::Vector3d>> points3D_board;
      std::map<uint32_t, Eigen::Vector3d> points3D_;
      cv::Ptr<cv::aruco::CharucoBoard> board_;
    };
  inline const std::map<uint32_t, Eigen::Vector3d>& CharucoBoardFeatureExtractor::Points3D() {
    return points3D_;
  }
  inline const std::map<uint32_t,std::map<uint32_t, Eigen::Vector3d>>& CharucoBoardFeatureExtractor::Points3D_withboard() {
    return points3D_board;
  }


}  // namespace calibmar