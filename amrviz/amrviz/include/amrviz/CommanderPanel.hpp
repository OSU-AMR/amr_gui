#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <QTimer>
#include <QMessageBox>
#include <QComboBox>
#include <QPushButton>
#include <QVariant>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "ui_CommanderPanel.h"
#include <string>
#include <vector>
#include <unordered_set>
#include <map>
#include <optional>

#include <yaml-cpp/yaml.h>

#define HEARTBEAT_STRING "heartbeat"
#define ROBOT_KILL_STATE_TOPIC_SUFFIX "/set_kill_state"
#define ROBOT_GENERAL_STATE_COMMAND_TOPIC_SUFFIX "/set_general_state_cmd"

#define PANEL_SELECTED_ROBOT_KILL_STATE_TOPIC "/commander_panel/selected_robot_kill_state"
#define PANEL_SELECTED_ROBOT_GENERAL_STATE_TOPIC "/commander_panel/selected_robot_general_state"

#define STATE_ERROR_UNKNOWN 255
#define STATE_CONTINUOUS_NONE 255

namespace amrviz {

struct RobotStateInfo {
    std::string display_name;
    uint8_t id;
    std::string yaml_key;
};

class CommanderPanel : public rviz_common::Panel {
    Q_OBJECT

public:
    CommanderPanel(QWidget *parent = 0);
    ~CommanderPanel();

    void load(const rviz_common::Config &config) override;
    void save(rviz_common::Config config) const override;
    void onInitialize() override;

    void refresh_robot_list();

public Q_SLOTS:
    void onEnableCurrentButtonToggled(bool checked);
    void onDisableCurrentButtonToggled(bool checked);
    // Slots for "All" buttons are for simple clicks, not toggles
    void onEnableAllButtonClicked();
    void onDisableAllButtonClicked();
    void onSetStateButtonClicked();
    void onRobotSelectionChanged(int index);
    void onFocusCheckBoxToggled(bool checked);

private:
    bool loadStatesFromYaml(const std::string& yaml_file_path);
    std::optional<uint8_t> getStateIdByYamlKey(const std::string& yaml_key) const;

    void updateButtonAndStatusUI(const std::string& robot_name);
    void updateStatusLabel(const std::string& robot_name, uint8_t state_code);
    void handleUiAfterRobotSelectionChange();

    rclcpp::Node::SharedPtr getNode();
    std::vector<std::string> getLiveRobotNamesFromHeartbeats();

    // Simplified: no longer needs is_all_command_scope, only for "Current" buttons
    void handleCheckableButtonPair(QPushButton* button_just_toggled, bool is_checked,
                                   QPushButton* other_button_in_pair,
                                   uint8_t command_if_button_checked);

    void sendDirectKillUnKillCommand(const std::string& robot_name, uint8_t command_id);
    void sendDirectGeneralCommand(const std::string& robot_name, uint8_t command_id);

    std::map<std::string, uint8_t> robot_commanded_generic_state_;
    std::map<std::string, uint8_t> robot_last_kill_unkill_cmd_;
    std::vector<RobotStateInfo> available_states_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr> robot_direct_kill_state_publishers_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr selected_robot_kill_state_1hz_publisher_;
    rclcpp::TimerBase::SharedPtr selected_robot_kill_state_1hz_timer_;
    std::optional<uint8_t> active_kill_unkill_for_selected_robot_1hz_;
    void publishActiveSelectedRobotKillState();

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr selected_robot_general_state_1hz_publisher_;
    rclcpp::TimerBase::SharedPtr selected_robot_general_state_1hz_timer_;
    std::optional<uint8_t> active_general_cmd_for_selected_robot_1hz_;
    void publishActiveSelectedRobotGeneralState();

public:
    Ui_CommanderPanel *uiPanel;
    rclcpp::TimerBase::SharedPtr robot_list_timer;

    std::vector<std::string> split_topics_into_componets(const std::string& str);
    void setComboBoxItems(QComboBox* comboBox, const std::vector<std::string>& items, const std::string& desired_selection);
    void removeDuplicateStrings(std::vector<std::string>& vec);
    double get_ros2_time();
    std::string get_current_robot();
};

} // namespace amrviz