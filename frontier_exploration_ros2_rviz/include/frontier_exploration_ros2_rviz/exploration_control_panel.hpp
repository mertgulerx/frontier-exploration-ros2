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

#pragma once

#include <memory>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>

#include <rviz_common/panel.hpp>

#include "frontier_exploration_ros2_rviz/exploration_panel_backend.hpp"

class QVBoxLayout;

namespace frontier_exploration_ros2_rviz
{

class ExplorationControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ExplorationControlPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void refresh_panel();
  void send_start();
  void send_stop();
  void handle_service_selection_changed(const QString & service_name);
  void handle_override_changed();

private:
  void build_ui();
  void update_snapshot_view();
  void sync_service_combo(const PanelSnapshot & snapshot);
  void set_info_value(QLabel * label, const QString & text) const;

  std::shared_ptr<ExplorationPanelBackend> backend_;

  QComboBox * service_combo_{nullptr};
  QLineEdit * node_override_edit_{nullptr};
  QDoubleSpinBox * delay_spin_{nullptr};
  QCheckBox * stop_quit_checkbox_{nullptr};
  QPushButton * start_button_{nullptr};
  QPushButton * stop_button_{nullptr};
  QLabel * service_info_value_{nullptr};
  QLabel * control_state_value_{nullptr};
  QLabel * last_command_value_{nullptr};
  QLabel * status_event_value_{nullptr};
  QLabel * countdown_value_{nullptr};
  QTimer * refresh_timer_{nullptr};
  QString last_saved_service_;
};

}  // namespace frontier_exploration_ros2_rviz
