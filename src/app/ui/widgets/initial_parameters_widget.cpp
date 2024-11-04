#include "initial_parameters_widget.h"

namespace calibmar {

  InitialParametersWidget::InitialParametersWidget(QWidget* parent, bool show_checkbox) : QWidget(parent),layout_main(new QVBoxLayout(this)),layout(new QHBoxLayout(this)), parameters_layout(new QVBoxLayout(this)),camNumSpinBox(new QSpinBox(this)){
    parameters_edit_ = new QLineEdit(this);
    parameters_edit_->setPlaceholderText("Initial Parameters");
    parameters_checkbox_ = new QCheckBox(this);

    connect(parameters_checkbox_, &QCheckBox::stateChanged, this,
            [this](int state) { parameters_edit_->setEnabled(parameters_checkbox_->isChecked()); });
    parameters_edit_->setEnabled(parameters_checkbox_->isChecked());
    parameters_checkbox_->setVisible(show_checkbox);

    // QVBoxLayout* layout_main = new QVBoxLayout(this);
    // QHBoxLayout* layout = new QHBoxLayout(this);
    // 添加 parameters_checkbox_
    layout->addWidget(parameters_checkbox_);
    // 添加 QSpinBox
    // QSpinBox* camNumSpinBox = new QSpinBox(this);
    camNumSpinBox->setRange(0, 10); // 设置范围
    camNumSpinBox->setValue(1); // 初始值
    camNumSpinBox->setFixedSize(20, 20);
    layout->addWidget(camNumSpinBox);
        // 设置布局间距和边距
    layout->setSpacing(5);
    layout->setContentsMargins(0, 0, 0, 0);
    // 连接 QSpinBox 的值变化信号

    connect(camNumSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &InitialParametersWidget::updateInitialParameters);
    // connect(camNumSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this,
    //           [&]() {
    //               emit updateInitialParameters(camNumSpinBox->value());
    //           });

    // QVBoxLayout* parameters_layout = new QVBoxLayout(this);
    parameters_layout->setContentsMargins(0, 0, 0, 0);
    parameters_layout->addWidget(parameters_edit_);
    layout_main->addLayout(layout);
    layout_main->addLayout(parameters_layout);

    layout_main->addStretch(); 
    count=camNumSpinBox->value();
  }

  void InitialParametersWidget::updateInitialParameters(int value) {
    count=value;
    QLayoutItem* item;
    while ((item= parameters_layout->takeAt(0)) != nullptr){
      delete item->widget();
      delete item;
    }
    // parameters_edit_->setPlaceholderText(QString("Parameter %1").arg(value));
    for (int i = 0; i < value; i++) {
      QLineEdit* new_edit = new QLineEdit(this);
      new_edit->setPlaceholderText(QString("Parameter cam%1").arg(i + 1));
      // new_edit->setEnabled(true); // 根据复选框设置启用状态
      new_edit->setVisible(true);
      parameters_layout->addWidget(new_edit);
    }
    return;
  }
  // void InitialParametersWidget::SetInitialParameters(const std::optional<std::string>& parameters) {
  //   parameters_checkbox_->setChecked(parameters.has_value());
  //   if (parameters.has_value()) {
  //     parameters_edit_->setText(QString::fromStdString(parameters.value()));
  //   }
  //   else {
  //     parameters_edit_->clear();
  //   }
  // }
  void InitialParametersWidget::SetInitialParameters(const std::vector<std::string>& parameters) {//liheng3
    parameters_checkbox_->setChecked(!parameters.empty());
    
    if (!parameters.empty()) {
      for (int i = 0; i < parameters.size(); ++i) {
        QLayoutItem* item = parameters_layout->itemAt(i); // 获取布局中的第 i 个项
        if (item) {
          QWidget* widget = item->widget(); // 获取该项的 widget
          QLineEdit* lineEdit = qobject_cast<QLineEdit*>(widget);
          if (lineEdit) {
            lineEdit->setText(QString::fromStdString(parameters[i]));
          }
        }
      }
    // else {
    //   parameters_edit_->clear();
    // }
    }
  }
  // std::optional<std::string> InitialParametersWidget::InitialParameters() {
  //   if (parameters_checkbox_->isChecked()) {
  //     return parameters_edit_->text().toStdString();
  //   }
  //   else {
  //     return {};
  //   }
  // }
  std::optional<std::string> InitialParametersWidget::InitialParameters(int i) {
    if (parameters_checkbox_->isChecked()) {
      if (i >= 0 && i < parameters_layout->count()) {
        QLayoutItem* item = parameters_layout->itemAt(i); // 获取布局中的第 i 个项
        if (item) {
          QWidget* widget = item->widget(); // 获取该项的 widget
          QLineEdit* lineEdit = qobject_cast<QLineEdit*>(widget);
          if (lineEdit) {
            return lineEdit->text().toStdString();
          }
        }
      }
    }
    else {
      return {};
    }
  }

  void InitialParametersWidget::SetChecked(bool checked) {
    parameters_checkbox_->setChecked(checked);
  }

  std::optional<int> InitialParametersWidget::Initialcount()  {
    // if (parameters_checkbox_->isChecked()) {
    //   return camNumSpinBox->value();
    // }
    // else {
    //   return {};
    // }
    return count;
  }

  void InitialParametersWidget::SetInitialcameranum(const std::optional<int>& count){
    if (!parameters_checkbox_ || !camNumSpinBox) {
      qDebug() << "Checkbox or SpinBox not initialized!";
      return;
    }
    parameters_checkbox_->setChecked(count.has_value());
    if (count.has_value()) {
      camNumSpinBox->setValue(count.value());
    } 
    else {
      parameters_checkbox_->setChecked(true);
      camNumSpinBox->setValue(1); 
    }
  }
}
