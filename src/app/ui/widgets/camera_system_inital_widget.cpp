#include "camera_system_inital_widget.h"

namespace calibmar {

  CameraSystemInitalWidget::CameraSystemInitalWidget(QWidget* parent) : QWidget(parent),layout(new QVBoxLayout(this)),formLayout_target_type(new QFormLayout(this)),camNumSpinBox(new QSpinBox(this)),rootCamSpinBox(new QSpinBox(this)),camNumLabel(new QLabel("Camera Num:", this)),rootCamLabel(new QLabel("Root Cam:", this)){
    // QLabel* camNumLabel = new QLabel(this);
    // QLabel* rootCamLabel = new QLabel(this);
    // camNumLabel->setText("Camera Num:");
    // rootCamLabel->setText("Root Num");
    // 添加 QSpinBox
    // QSpinBox*  camNumSpinBox = new QSpinBox(this);
    camNumSpinBox->setRange(0, 10); // 设置范围
    camNumSpinBox->setValue(1); // 初始值
    camNumSpinBox->setSingleStep(1);
    // camNumSpinBox->setFixedSize(20, 20);

    // QSpinBox* rootCamSpinBox = new QSpinBox(this);
    rootCamSpinBox->setRange(0, 10);
    rootCamSpinBox->setValue(1); // 初始值
    rootCamSpinBox->setSingleStep(1);

    // QFormLayout* formLayout_target_type=new QFormLayout(this);
    formLayout_target_type->setWidget(1, QFormLayout::LabelRole, camNumLabel);
    formLayout_target_type->setWidget(1, QFormLayout::FieldRole, camNumSpinBox);
    formLayout_target_type->setWidget(2, QFormLayout::LabelRole, rootCamLabel);
    formLayout_target_type->setWidget(2, QFormLayout::FieldRole, rootCamSpinBox);
   
    // QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addLayout(formLayout_target_type);
        // 设置布局间距和边距
    // layout->setSpacing(5);
    layout->setContentsMargins(0, 0, 0, 10);
    // 连接 QSpinBox 的值变化信号
    connect(camNumSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraSystemInitalWidget::updateInitialParameters);
    connect(rootCamSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraSystemInitalWidget::updaterootcam);
    // layout->addStretch(); 
    count=camNumSpinBox->value();
    root=rootCamSpinBox->value();
  }

  void CameraSystemInitalWidget::updateInitialParameters(int value) {
    count=value;
    return;
  }
  void CameraSystemInitalWidget::updaterootcam(int value) {
    root=value;
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
  void CameraSystemInitalWidget::SetInitialParameters(const std::vector<std::string>& parameters) {//liheng3
    // parameters_checkbox_->setChecked(!parameters.empty());
    
    // if (!parameters.empty()) {
    //   for (int i = 0; i < parameters.size(); ++i) {
    //     QLayoutItem* item = parameters_layout->itemAt(i); // 获取布局中的第 i 个项
    //     if (item) {
    //       QWidget* widget = item->widget(); // 获取该项的 widget
    //       QLineEdit* lineEdit = qobject_cast<QLineEdit*>(widget);
    //       if (lineEdit) {
    //         lineEdit->setText(QString::fromStdString(parameters[i]));
    //       }
    //     }
    //   }
    // else {
    //   parameters_edit_->clear();
    // }
    // }
  }
  // std::optional<std::string> InitialParametersWidget::InitialParameters() {
  //   if (parameters_checkbox_->isChecked()) {
  //     return parameters_edit_->text().toStdString();
  //   }
  //   else {
  //     return {};
  //   }
  // }
  std::optional<std::string> CameraSystemInitalWidget::InitialParameters(int i) {
    // if (parameters_checkbox_->isChecked()) {
    //   if (i >= 0 && i < parameters_layout->count()) {
    //     QLayoutItem* item = parameters_layout->itemAt(i); // 获取布局中的第 i 个项
    //     if (item) {
    //       QWidget* widget = item->widget(); // 获取该项的 widget
    //       QLineEdit* lineEdit = qobject_cast<QLineEdit*>(widget);
    //       if (lineEdit) {
    //         return lineEdit->text().toStdString();
    //       }
    //     }
    //   }
    // }
    // else {
      return {};
    // }
  }

  void CameraSystemInitalWidget::SetChecked(bool checked) {
    // parameters_checkbox_->setChecked(checked);
  }

  std::optional<int> CameraSystemInitalWidget::Initialcount()  {
    // if (parameters_checkbox_->isChecked()) {
    return camNumSpinBox->value();
    // }
    // else {
    //   return {};
    // }
    // return count;
  }

  std::optional<int> CameraSystemInitalWidget::Initialrootcam()  {
    // if (parameters_checkbox_->isChecked()) {
    //   return camNumSpinBox->value();
    // }
    // else {
    //   return {};
    // }
    return root;
  }

  void CameraSystemInitalWidget::SetInitialcameranum(const std::optional<int>& count){
    if (!camNumSpinBox) {
      qDebug() << "Checkbox or SpinBox not initialized!";
      return;
    }
    if (count.has_value()) {
      camNumSpinBox->setValue(count.value());
    } 
    else {
      camNumSpinBox->setValue(1); 
    }
  }
  void CameraSystemInitalWidget::SetInitialrootcam(const std::optional<int>& root){
    if (!rootCamSpinBox) {
      qDebug() << "Checkbox or SpinBox not initialized!";
      return;
    }
    if (root.has_value()) {
      rootCamSpinBox->setValue(root.value());
    } 
    else {
      rootCamSpinBox->setValue(2); 
    }
  }
}
