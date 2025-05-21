#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <QTimer>
#include <QMessageBox>
#include <QComboBox>
#include <QPushButton>
#include <QVariant>
#include <std_msgs/msg/u_int8.hpp>
#include "ui_CommanderPanel.h" // Assuming this is correctly generated
#include <string>
#include <vector>
#include <unordered_set>
#include <map>
#include <optional>

#include <yaml-cpp/yaml.h>
#include <amr_msgs/srv/state_change_request.hpp> // Re-added for the service client

#define HEARTBEAT_STRING "heartbeat"
#define ROBOT_STATE_CHANGE_SERVICE_SUFFIX "/request_state_change" // Will be used for the Set State button

#define STATE_ERROR_UNKNOWN 255
#define STATE_CONTINUOUS_NONE 255

namespace amrviz
{

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

public Q_SLOTS:
    void refresh_robot_list();
    void onEnableCurrentButtonClicked();
    void onDisableCurrentButtonClicked();
    void onEnableAllButtonClicked();
    void onDisableAllButtonClicked();
    void onSetStateButtonClicked(); // This will use a service client
    void onRobotSelectionChanged(int index);
    void onFocusCheckBoxToggled(bool checked);

private:
    void publish_kill_states();

    bool loadStatesFromYaml(const std::string& yaml_file_path);
    std::optional<uint8_t> getStateIdByYamlKey(const std::string& yaml_key) const;

    void updateButtonAndStatusUI(const std::string& robot_name);
    void updateStatusLabel(const std::string& robot_name, uint8_t state_code);
    void handleUiAfterRobotSelectionChange();

    rclcpp::Node::SharedPtr getNode();
    std::vector<std::string> getLiveRobotNamesFromHeartbeats();

    // This specific method for general commands via service is reinstated in principle
    // void sendDirectGeneralCommandViaService(const std::string& robot_name, uint8_t command_id); // Example name

    std::map<std::string, uint8_t> robot_commanded_generic_state_;
    std::map<std::string, uint8_t> robot_last_kill_unkill_cmd_;
    std::vector<RobotStateInfo> available_states_;

    std::map<std::string, rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr> robot_kill_state_publishers_;
    std::map<std::string, uint8_t> robot_desired_kill_states_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr selected_robot_command_publisher_; // For enable/disable current
    rclcpp::TimerBase::SharedPtr kill_state_publish_timer_;

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