/*
Copyright 2026 Mert Güler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "frontier_exploration_ros2_rviz/exploration_control_panel.hpp"

#include <memory>

#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace frontier_exploration_ros2_rviz
{

ExplorationControlPanel::ExplorationControlPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  build_ui();
}

void ExplorationControlPanel::onInitialize()
{
  rviz_common::Panel::onInitialize();

  auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    return;
  }

  backend_ = std::make_shared<ExplorationPanelBackend>(ros_node_abstraction->get_raw_node());
  refresh_timer_->start(250);
  refresh_panel();
}

void ExplorationControlPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  QString selected_service;
  if (config.mapGetString("SelectedService", &selected_service)) {
    last_saved_service_ = selected_service;
  }

  QString override_value;
  if (config.mapGetString("NodeOverride", &override_value)) {
    node_override_edit_->setText(override_value);
  }

  float delay = 0.0F;
  if (config.mapGetFloat("DelaySeconds", &delay)) {
    delay_spin_->setValue(delay);
  }

  bool quit_after_stop = false;
  if (config.mapGetBool("QuitAfterStop", &quit_after_stop)) {
    stop_quit_checkbox_->setChecked(quit_after_stop);
  }
}

void ExplorationControlPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("SelectedService", service_combo_->currentText());
  config.mapSetValue("NodeOverride", node_override_edit_->text());
  config.mapSetValue("DelaySeconds", delay_spin_->value());
  config.mapSetValue("QuitAfterStop", stop_quit_checkbox_->isChecked());
}

void ExplorationControlPanel::refresh_panel()
{
  if (!backend_) {
    return;
  }

  backend_->set_delay_seconds(delay_spin_->value());
  backend_->set_quit_after_stop(stop_quit_checkbox_->isChecked());
  backend_->poll();
  update_snapshot_view();
}

void ExplorationControlPanel::send_start()
{
  if (!backend_) {
    return;
  }

  backend_->set_delay_seconds(delay_spin_->value());
  backend_->send_start();
  refresh_panel();
}

void ExplorationControlPanel::send_stop()
{
  if (!backend_) {
    return;
  }

  backend_->set_delay_seconds(delay_spin_->value());
  backend_->set_quit_after_stop(stop_quit_checkbox_->isChecked());
  backend_->send_stop();
  refresh_panel();
}

void ExplorationControlPanel::handle_service_selection_changed(const QString & service_name)
{
  if (backend_) {
    backend_->set_selected_service(service_name.toStdString());
  }
}

void ExplorationControlPanel::handle_override_changed()
{
  if (backend_) {
    backend_->set_logger_or_node_override(node_override_edit_->text().toStdString());
  }
}

void ExplorationControlPanel::build_ui()
{
  auto * root_layout = new QVBoxLayout(this);
  root_layout->setContentsMargins(6, 6, 6, 6);

  auto * target_group = new QGroupBox("Explorer Target", this);
  auto * target_layout = new QFormLayout(target_group);
  service_combo_ = new QComboBox(target_group);
  service_combo_->setEditable(false);
  node_override_edit_ = new QLineEdit(target_group);
  node_override_edit_->setPlaceholderText("frontier_explorer or /ns/frontier_explorer");
  target_layout->addRow("Service", service_combo_);
  target_layout->addRow("Node Override", node_override_edit_);
  root_layout->addWidget(target_group);

  auto * control_group = new QGroupBox("Control", this);
  auto * control_layout = new QHBoxLayout(control_group);
  delay_spin_ = new QDoubleSpinBox(control_group);
  delay_spin_->setDecimals(1);
  delay_spin_->setRange(0.0, 36000.0);
  delay_spin_->setSingleStep(0.5);
  delay_spin_->setSuffix(" s");
  start_button_ = new QPushButton("Start", control_group);
  stop_button_ = new QPushButton("Stop", control_group);
  stop_quit_checkbox_ = new QCheckBox("Stop with -q", control_group);
  control_layout->addWidget(new QLabel("Delay", control_group));
  control_layout->addWidget(delay_spin_);
  control_layout->addWidget(start_button_);
  control_layout->addWidget(stop_button_);
  control_layout->addWidget(stop_quit_checkbox_);
  root_layout->addWidget(control_group);

  auto * info_group = new QGroupBox("Info", this);
  auto * info_layout = new QFormLayout(info_group);
  service_info_value_ = new QLabel(info_group);
  control_state_value_ = new QLabel(info_group);
  last_command_value_ = new QLabel(info_group);
  status_event_value_ = new QLabel(info_group);
  countdown_value_ = new QLabel(info_group);
  set_info_value(service_info_value_, "No control service discovered");
  set_info_value(control_state_value_, "idle");
  set_info_value(last_command_value_, "No command sent");
  set_info_value(status_event_value_, "Waiting for frontier explorer events");
  set_info_value(countdown_value_, "");
  info_layout->addRow("Service", service_info_value_);
  info_layout->addRow("Control State", control_state_value_);
  info_layout->addRow("Last Command", last_command_value_);
  info_layout->addRow("Status/Event", status_event_value_);
  info_layout->addRow("Countdown", countdown_value_);
  root_layout->addWidget(info_group);
  root_layout->addStretch(1);

  refresh_timer_ = new QTimer(this);
  refresh_timer_->setSingleShot(false);

  connect(refresh_timer_, &QTimer::timeout, this, &ExplorationControlPanel::refresh_panel);
  connect(start_button_, &QPushButton::clicked, this, &ExplorationControlPanel::send_start);
  connect(stop_button_, &QPushButton::clicked, this, &ExplorationControlPanel::send_stop);
  connect(
    service_combo_,
    &QComboBox::currentTextChanged,
    this,
    &ExplorationControlPanel::handle_service_selection_changed);
  connect(node_override_edit_, &QLineEdit::editingFinished, this, &ExplorationControlPanel::handle_override_changed);
}

void ExplorationControlPanel::update_snapshot_view()
{
  const PanelSnapshot snapshot = backend_->snapshot();
  sync_service_combo(snapshot);

  set_info_value(service_info_value_, QString::fromStdString(snapshot.service_text));
  set_info_value(control_state_value_, QString::fromStdString(snapshot.control_state_text));
  set_info_value(last_command_value_, QString::fromStdString(snapshot.last_command_text));
  set_info_value(countdown_value_, QString::fromStdString(
      snapshot.countdown_text.empty() ? "-" : snapshot.countdown_text));

  const QString event_text = QString::fromStdString(snapshot.status_event_text);
  const QString event_color = QString::fromStdString(panel_event_color_to_hex(snapshot.status_color));
  status_event_value_->setText(event_text);
  status_event_value_->setWordWrap(true);
  status_event_value_->setStyleSheet(
    QString("QLabel { background-color: %1; color: white; padding: 4px; border-radius: 4px; }")
    .arg(event_color));

  start_button_->setEnabled(snapshot.service_ready);
  stop_button_->setEnabled(snapshot.service_ready);
  service_combo_->setVisible(snapshot.discovered_services.size() > 1U);
}

void ExplorationControlPanel::sync_service_combo(const PanelSnapshot & snapshot)
{
  const QSignalBlocker blocker(service_combo_);
  const QString current_selection = service_combo_->currentText();

  service_combo_->clear();
  for (const auto & service_name : snapshot.discovered_services) {
    service_combo_->addItem(QString::fromStdString(service_name));
  }

  QString desired_selection = QString::fromStdString(snapshot.selected_service);
  if (desired_selection.isEmpty() && !last_saved_service_.isEmpty()) {
    desired_selection = last_saved_service_;
  }

  const int index = service_combo_->findText(desired_selection);
  if (index >= 0) {
    service_combo_->setCurrentIndex(index);
  } else if (!current_selection.isEmpty()) {
    const int current_index = service_combo_->findText(current_selection);
    if (current_index >= 0) {
      service_combo_->setCurrentIndex(current_index);
    }
  }

  if (!snapshot.node_override.empty() && node_override_edit_->text().isEmpty()) {
    node_override_edit_->setText(QString::fromStdString(snapshot.node_override));
  }
}

void ExplorationControlPanel::set_info_value(QLabel * label, const QString & text) const
{
  label->setText(text);
  label->setWordWrap(true);
  label->setStyleSheet("QLabel { padding: 2px 0px; }");
}

}  // namespace frontier_exploration_ros2_rviz

PLUGINLIB_EXPORT_CLASS(
  frontier_exploration_ros2_rviz::ExplorationControlPanel,
  rviz_common::Panel)
