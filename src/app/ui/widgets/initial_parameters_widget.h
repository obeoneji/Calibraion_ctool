#pragma once

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class InitialParametersWidget : public QWidget {

  public:
    InitialParametersWidget(QWidget* parent = nullptr, bool show_checkbox = true);
    void SetInitialParameters(const std::vector<std::string>& parameters);
    void SetInitialcameranum(const std::optional<int>& count);
    std::optional<std::string> InitialParameters(int i);
    void SetChecked(bool checked);
    void updateInitialParameters(int value);
    std::optional<int> Initialcount();
    std::optional<int> count;
  private:
    // std::optional<int> conut=1;
    QSpinBox* camNumSpinBox;
    QVBoxLayout* layout_main;
    QLineEdit* parameters_edit_;
    QCheckBox* parameters_checkbox_;
    QVBoxLayout* parameters_layout;
    QHBoxLayout* layout;
  };
}
