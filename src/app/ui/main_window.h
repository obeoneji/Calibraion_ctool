#pragma once

#include "calibmar/core/pixmap.h"
#include "calibmar/extractors/chessboard_extractor.h"
#include "ui/dialogs/file_calibration_dialog.h"
#include "ui/dialogs/calibration_system_dialog.h"
#include "ui/dialogs/stereo_file_calibration_dialog.h"
#include "ui/dialogs/stream_calibration_dialog.h"
#include "ui/widgets/calibration_widget.h"
#include "ui/widgets/livestream_extraction_widget.h"

#include <QImage>
#include <QLabel>
#include <QMainWindow>
#include <QScrollArea>
#include <opencv2/core.hpp>
#include <vector>

namespace calibmar {
  class MainWindow : public QMainWindow {
   public:
    MainWindow(QWidget* parent = nullptr);

   private:
    // void NewStreamCalibration();
    void NewFilesCalibration();
    void CalibrationSystem();
    // void NewStereoFilesCalibration();
    void SaveCalibration();
    void About();
    void CreateActions();
    void BeginNewCalibration();
    void EndNewCalibration();

    void DisplayExtractionImage(const std::string& image_name, const TargetVisualizer& target_visualizer);

    QScrollArea* scroll_area_;
    QVBoxLayout* main_layout_;
    QAction* calibration_files_action_;
    QAction* calibration_system_action_;
    // QAction* calibration_stereo_files_action_;
    // QAction* calibration_stream_action_;
    QAction* calibration_save_action_;
    std::string last_directory_;
    std::string last_output_directory_;
    std::optional<int> cam_num;
    std::unique_ptr<std::thread> worker_thread_;
    std::vector<std::unique_ptr<std::thread>> worker_threads; // 存储多个线程
    std::unique_ptr<Calibration> calibration_;
    std::vector<std::unique_ptr<Calibration>> calibrations_;
    std::atomic<int> successful_calibrations{0};
    // second calibration that only exists for a stereo calibration
    // std::unique_ptr<Calibration> calibration_stereo_;
    std::atomic_bool calibration_success_;
    bool scrolled_to_bottom_;
    // to persist dialog options between runs
    FileCalibrationDialog::Options file_calibration_options_;
    CalibrationSystemDialog::Options calibration_system_options_;
    // StreamCalibrationDialog::Options stream_calibration_options_;
    // StereoFileCalibrationDialog::Options stereo_calibration_options_;
  };
}
