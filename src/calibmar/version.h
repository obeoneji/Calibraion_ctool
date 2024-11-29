#pragma once

#include <string>

namespace calibmar {

  const static std::string CALIBMAR_VERSION = "1.0";
  const static std::string CALIBMAR_COMMIT_ID = "051cb40";
  const static std::string CALIBMAR_COMMIT_DATE = "2024-11-05";
  const static std::string CALIBMAR_PURPOSE = "This software is intended for academic research purposes only. ";
#if defined(COLMAP_CUDA_ENABLED)
  const static std::string CALIBMAR_CUDA_ENABLED = "CUDA enabled";
#else
  const static std::string CALIBMAR_CUDA_ENABLED = "CUDA disabled";
#endif
  const static int CALIBMAR_VERSION_MAJOR = 1;
  const static int CALIBMAR_VERSION_MINOR = 0;
}
