#pragma once

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class CameraSystemInitalWidget : public QWidget {

  public:
    CameraSystemInitalWidget(QWidget* parent = nullptr);
    void SetInitialParameters(const std::vector<std::string>& parameters);
    void SetInitialcameranum(const std::optional<int>& count);
    void SetInitialrootcam(const std::optional<int>& root);
    std::optional<std::string> InitialParameters(int i);
    void SetChecked(bool checked);
    void updateInitialParameters(int value);
    void updaterootcam(int value);
    std::optional<int> Initialcount();
    std::optional<int> Initialrootcam();
    std::optional<int> count;
    std::optional<int> root;
  private:
    // std::optional<int> conut=1;
    QLabel*  camNumLabel;
    QLabel*  rootCamLabel;
    QSpinBox* camNumSpinBox;
    QSpinBox* rootCamSpinBox;
    QCheckBox* parameters_checkbox_;
    QFormLayout* formLayout_target_type;
    QVBoxLayout* layout;
  };
}
