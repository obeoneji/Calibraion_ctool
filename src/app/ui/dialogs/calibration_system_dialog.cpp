#include "calibration_system_dialog.h"

#include <calibmar/core/report.h>
#include <colmap/util/misc.h>
#include <filesystem>

namespace calibmar {

  CalibrationSystemDialog::CalibrationSystemDialog(QWidget* parent) : QDialog(parent) {
    // directory groupbox
    QGroupBox* directory_groupbox = new QGroupBox(this);
    directory_groupbox->setTitle("Image files directory");
    directory_edit_ = new QLineEdit(directory_groupbox);
    QPushButton* select_directory_button = new QPushButton(directory_groupbox);
    select_directory_button->setText("Browse");
    connect(select_directory_button, &QPushButton::released, this, [this]() {
      this->directory_edit_->setText(QFileDialog::getExistingDirectory(this, "Select calibration target image directory"));
    });
    QHBoxLayout* horizontal_layout_directory = new QHBoxLayout(directory_groupbox);
    horizontal_layout_directory->addWidget(directory_edit_);
    horizontal_layout_directory->addWidget(select_directory_button);
    // output directory groupbox
    QGroupBox* output_directory = new QGroupBox(this);
    output_directory->setTitle("Output undistorted image files directory");
    output_directory_edit_ = new QLineEdit(output_directory);
    QPushButton* output_directory_button = new QPushButton(output_directory);
    output_directory_button->setText("Browse");
    connect(output_directory_button, &QPushButton::released, this, [this]() {
      this->output_directory_edit_->setText(QFileDialog::getExistingDirectory(this, "Select output undistorted images directory"));
    });
    QHBoxLayout* horizontal_layout_output = new QHBoxLayout(output_directory);
    horizontal_layout_output->addWidget(output_directory_edit_);
    horizontal_layout_output->addWidget(output_directory_button);
    //g2o options groupbox
    QGroupBox* g2o_options = new QGroupBox(this);
    g2o_options->setTitle("Use g2o library to do bundle adjustment");

    g2o_solver_label = new QLabel(this);
    g2o_solver_label->setText("g2o solver type");
    g2o_solver_combobox_ = new QComboBox(this);
    g2o_solver_combobox_->addItem("Eigen");
    g2o_solver_combobox_->addItem("Cholmod");
    g2o_solver_combobox_->addItem("Dense");
    g2o_solver_combobox_->setCurrentIndex(1);
    QFormLayout* formLayout_solver_type = new QFormLayout();
    formLayout_solver_type->setWidget(0, QFormLayout::LabelRole, g2o_solver_label);
    formLayout_solver_type->setWidget(0, QFormLayout::FieldRole, g2o_solver_combobox_);

    g2o_checkbox_ = new QCheckBox(this);
    // 连接复选框状态变化以启用/禁用求解器选项

    connect(g2o_checkbox_, &QCheckBox::stateChanged, this,[this](int state){
      g2o_solver_label->setEnabled(g2o_checkbox_->isChecked());
      g2o_solver_combobox_->setEnabled(g2o_checkbox_->isChecked());
      g2o_solver_combobox_->setCurrentIndex(1);
    });
        // 初始状态禁用选项
    g2o_checkbox_->setVisible(true);
    g2o_solver_label->setEnabled(false);
    g2o_solver_combobox_->setEnabled(false);
    
    QHBoxLayout* horizontal_layout_g2o = new QHBoxLayout(g2o_options);

    horizontal_layout_g2o->addWidget(g2o_checkbox_);
    horizontal_layout_g2o->addWidget(g2o_solver_label);
    horizontal_layout_g2o->addWidget(g2o_solver_combobox_);


    // common options
    calibration_system_options_widget_ = new CalibrationSystemOptionsWidget(this);

    // import button
    QHBoxLayout* horizontalLayout_run = new QHBoxLayout();
    QPushButton* import_button = new QPushButton(this);
    import_button->setText("Import...");
    connect(import_button, &QPushButton::released, this, [this]() { ImportParameters(); });
    horizontalLayout_run->addWidget(import_button, 0, Qt::AlignLeft | Qt::AlignTop);

    // run button
    QPushButton* run_button = new QPushButton(this);
    run_button->setText("Run");
    run_button->setDefault(true);
    connect(run_button, &QPushButton::released, this, [this]() {
      if (Validate()) {
        this->accept();
      }
    });

    horizontalLayout_run->addWidget(run_button, 0, Qt::AlignRight | Qt::AlignTop);

    // main layout
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(directory_groupbox);
    layout->addWidget(output_directory);
    layout->addWidget(g2o_options);
    layout->addWidget(calibration_system_options_widget_);
    layout->addLayout(horizontalLayout_run);
    setWindowTitle("Calibrate Intrinsic and Extrinsic");
    // setMinimumSize(1000, 1000); // 根据需要调整这个值
    layout->setSizeConstraint(QLayout::SetMinimumSize);
  }

  void CalibrationSystemDialog::SetOptions(Options options) {
    directory_edit_->setText(QString::fromStdString(options.images_directory));
    output_directory_edit_->setText(QString::fromStdString(options.output_images_directory));
    g2o_checkbox_->setChecked(options.g2o_options);
    g2o_solver_combobox_->setCurrentIndex(options.solver_index); 
    CalibrationSystemOptionsWidget::Options calibration_options;
    calibration_options.calibration_target_options = options.calibration_target_options;
    calibration_options.camera_model = options.camera_model;
    calibration_options.cam_num = options.cam_num;
    calibration_options.root_cam = options.root_cam;
    calibration_system_options_widget_->SetOptions(calibration_options);
  }

  CalibrationSystemDialog::Options CalibrationSystemDialog::GetOptions() {
    CalibrationSystemOptionsWidget::Options calibration_options = calibration_system_options_widget_->GetOptions();
    Options options;
    options.camera_model = calibration_options.camera_model;
    options.calibration_target_options = calibration_options.calibration_target_options;
    options.images_directory = directory_edit_->text().toStdString();
    options.output_images_directory=output_directory_edit_->text().toStdString();
    options.g2o_options=g2o_checkbox_->isChecked();
    options.cam_num=calibration_options.cam_num;
    options.root_cam=calibration_options.root_cam;
    options.solver_index=g2o_solver_combobox_->currentIndex();
    return options;
  }

  bool CalibrationSystemDialog::Validate() {
    if (!std::filesystem::is_directory(directory_edit_->text().toStdString())) {
      QMessageBox::information(this, "Validation Error", "Image directory does not exist.");
      return false;
    }
    if (!std::filesystem::is_directory(output_directory_edit_->text().toStdString())) {
      QMessageBox::information(this, "Validation Error", "Output image directory does not exist.");
      return false;
    }

    return calibration_system_options_widget_->Validate();
  }

  void CalibrationSystemDialog::ImportParameters() {
    std::string path =
        QFileDialog::getOpenFileName(this, "Import Parameters", QString(), "Calibration YAML (*.yaml *.yml)").toStdString();
    if (path.empty()) {
      return;
    }

    ImportedParameters p = ImportedParameters::ImportFromYaml(path);
    Options options;
    switch (p.calibration_target) {
      case CalibrationTargetType::Chessboard:
        options.calibration_target_options = ChessboardFeatureExtractor::Options{p.rows, p.columns, p.square_size};
        break;
      case CalibrationTargetType::Target3D:
        options.calibration_target_options = SiftFeatureExtractor::Options{};
        break;
      case CalibrationTargetType::Target3DAruco:
        options.calibration_target_options = ArucoSiftFeatureExtractor::Options{p.aruco_type, p.aruco_scale_factor, false};
        break;
      case CalibrationTargetType::ArucoGridBoard:
        options.calibration_target_options = ArucoBoardFeatureExtractor::Options{p.aruco_type,
                                                                                 ArucoGridOrigin::TopLeft,
                                                                                 ArucoGridDirection::Horizontal,
                                                                                 p.columns,
                                                                                 p.rows,
                                                                                 p.square_size,
                                                                                 p.spacing,
                                                                                 1};
      case CalibrationTargetType::CharucoBoard:
        options.calibration_target_options = CharucoBoardFeatureExtractor::Options{p.aruco_type,
                                                                                 p.rows,
                                                                                 p.columns,
                                                                                 p.board_index,
                                                                                 p.marker_num,
                                                                                 p.square_size,
                                                                                 p.marker_size,
                                                                                 p.frame_num,
                                                                                 };

        break;
    }

    options.camera_model = p.camera_model;
    options.g2o_options=p.g2o_options;
    options.images_directory = p.directory;
    options.cam_num.emplace(p.camnum);
    options.solver_index=p.solver_index;
    SetOptions(options);
  }
}