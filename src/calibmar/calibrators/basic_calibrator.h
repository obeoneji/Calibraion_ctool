#pragma once

#include "calibmar/core/calibration.h"
#include "calibmar/core/camera_models.h"
#include "calibmar/extractors/extractor.h"
#include "calibrator.h"

#include <optional>

namespace calibmar {
  // Calibrator class calibrates the camera using contained 2D-3D correspondences.
  class BasicCalibrator : public Calibrator {
   public:
    struct Options {
      CameraModelType camera_model = CameraModelType::OpenCVCameraModel;
      // If true, the calibration camera model will be used as an initial intrinsics guess as is.
      bool use_intrinsics_guess = false;
      // Image size in pixel (width, height)
      std::pair<int, int> image_size = {0, 0};
      // Use LU instead of SVD decomposition for solving. Faster but potentially less precise (from opencv).
      bool fast = false;
      int cam_num=1;
      int board_num=1;
      int marker_num=1;
      int root_cam=1;
      int frame_num=1;
      int solver_index=0;
      int pattern=1;
      void Check();
    };

    BasicCalibrator(const Options& options);

    // Calibrates using the 3D points and images. The calibration camera, image poses, estimated parameters
    // standard deviations and reprojection error will be set.
    //
    // If the camera is already initialized and matches the target type it will be used for an initial guess of the
    // calibration.
    //
    // @param calibration Calibration. Must contain 3D Points and images containing corresponding 2D points.
    void Calibrate(Calibration& calibration) override;
   private:
    Options options_;
  };
}