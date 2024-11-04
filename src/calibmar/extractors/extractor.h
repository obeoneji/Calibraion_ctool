#pragma once

#include "calibmar/core/image.h"
#include "calibmar/core/pixmap.h"
#include <Eigen/Core>
#include <map>

namespace calibmar {

  // Abstract calibration target point extractor class
  class FeatureExtractor {
   public:
    enum class Status { SUCCESS, DETECTION_ERROR ,LACK_ERROR};

    virtual ~FeatureExtractor() = default;

    virtual Status Extract(Image& image, const Pixmap& pixmap) = 0;
  };
}