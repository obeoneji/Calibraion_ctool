#pragma once

#include "calibmar/core/calibration.h"
#include "ui/dialogs/calibration_system_dialog.h"
#include "ui/widgets/calibration_widget.h"

namespace calibmar {

  class SystemCalibrationRunner {
   public:
    SystemCalibrationRunner(CalibrationWidget* calibration_widget, CalibrationSystemDialog::Options options);

    bool Run(Calibration& calibration);

   private:
    CalibrationWidget* calibration_widget_;
    CalibrationSystemDialog::Options options_;

    std::unique_ptr<Pixmap> last_pixmap_;
  };
}
