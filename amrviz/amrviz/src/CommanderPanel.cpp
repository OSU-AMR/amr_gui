#include "amrviz/CommanderPanel.hpp"

#include <chrono>
#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <QComboBox>
#include <QMessageBox>
#include <QPushButton>
#include <QCheckBox>
#include <QVariant>
#include <algorithm>
#include <fstream>
#include <rclcpp/parameter.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <amr_msgs/srv/state_change_request.hpp> // Ensure this is included

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace amrviz
{

// Constructor and other initializations remain the same as the previous version
// where enable/disable buttons use publishers.
CommanderPanel::CommanderPanel(QWidget *parent) : rviz_common::Panel(parent)
{
    setFocusPolicy(Qt::ClickFocus);
    uiPanel = new Ui_CommanderPanel();
    uiPanel->setupUi(this);

    if (uiPanel->enableCurrentButton) {
        connect(uiPanel->enableCurrentButton, &QPushButton::clicked, this, &CommanderPanel::onEnableCurrentButtonClicked);
    } else { RVIZ_COMMON_LOG_ERROR("CmdPanel: enableCurrentButton is null."); }

    if (uiPanel->disableCurrentButton) {
        connect(uiPanel->disableCurrentButton, &QPushButton::clicked, this, &CommanderPanel::onDisableCurrentButtonClicked);
    } else { RVIZ_COMMON_LOG_ERROR("CmdPanel: disableCurrentButton is null."); }

    if (uiPanel->enableAllButton) {
        connect(uiPanel->enableAllButton, &QPushButton::clicked, this, &CommanderPanel::onEnableAllButtonClicked);
    } else { RVIZ_COMMON_LOG_WARNING("CmdPanel: enableAllButton is null."); }

    if (uiPanel->disableAllButton) {
        connect(uiPanel->disableAllButton, &QPushButton::clicked, this, &CommanderPanel::onDisableAllButtonClicked);
    } else { RVIZ_COMMON_LOG_WARNING("CmdPanel: disableAllButton is null."); }
    
    if (uiPanel->setStateButton) {
        connect(uiPanel->setStateButton, &QPushButton::clicked, this, &CommanderPanel::onSetStateButtonClicked);
    } else { RVIZ_COMMON_LOG_WARNING("CmdPanel: setStateButton is null."); }

    if (uiPanel->robot_select_block) {
       connect(uiPanel->robot_select_block, QOverload<int>::of(&QComboBox::currentIndexChanged),
               this, &CommanderPanel::onRobotSelectionChanged);
    } else { RVIZ_COMMON_LOG_WARNING("CmdPanel: robot_select_block is null."); }

    if (uiPanel->focus_box) {
        connect(uiPanel->focus_box, &QCheckBox::toggled, this, &CommanderPanel::onFocusCheckBoxToggled);
    } else { RVIZ_COMMON_LOG_WARNING("CmdPanel: focus_box is null.");}

    RVIZ_COMMON_LOG_INFO("CommanderPanel: Constructed panel.");
}

CommanderPanel::~CommanderPanel()
{
    delete uiPanel;
}

// YAML Parsing methods (parseStatesInCategory, loadStatesFromYaml, getStateIdByYamlKey) remain the same.
void parseStatesInCategory(const YAML::Node& category_node, const std::string& category_name, std::vector<RobotStateInfo>& states_vector, rclcpp::Logger logger) {
    if (!category_node || !category_node.IsMap()) { 
        RCLCPP_WARN(logger, "Category '%s' is not valid in YAML or does not exist.", category_name.c_str()); 
        return; 
    }
    for (YAML::const_iterator it = category_node.begin(); it != category_node.end(); ++it) {
        std::string key = it->first.as<std::string>(); YAML::Node details = it->second;
        if (!details["id"]) { RCLCPP_WARN(logger, "State '%s' in '%s' missing 'id'.", key.c_str(), category_name.c_str()); continue; }
        try {
            uint8_t id = details["id"].as<uint8_t>(); std::string d_name = key;
            std::replace(d_name.begin(), d_name.end(), '_', ' ');
            if(!d_name.empty()) d_name[0] = toupper(d_name[0]);
            for(size_t i = 1; i < d_name.length(); ++i) if(d_name[i-1] == ' ') d_name[i] = toupper(d_name[i]);
            states_vector.push_back({d_name + " (" + std::to_string(id) + ")", id, key});
        } catch (const YAML::Exception& e) { RCLCPP_ERROR(logger, "YAML parse error for state '%s': %s", key.c_str(), e.what()); }
    }
}

bool CommanderPanel::loadStatesFromYaml(const std::string& yaml_path) { 
    auto node = getNode(); if (!node) return false; rclcpp::Logger logger = node->get_logger();
    RCLCPP_INFO(logger, "Loading states from YAML: %s", yaml_path.c_str());
    std::ifstream f(yaml_path.c_str()); 
    if (!f.good()) { 
        RCLCPP_ERROR(logger, "State YAML file not found/readable: %s", yaml_path.c_str());
        QMessageBox::warning(this, "YAML Error", QString("State file not found: %1").arg(QString::fromStdString(yaml_path))); 
        return false; 
    } 
    f.close();
    available_states_.clear();
    try {
        YAML::Node config = YAML::LoadFile(yaml_path); if (!config) { /* error handling */ return false; } //
        parseStatesInCategory(config["bootup_states"], "bootup_states", available_states_, logger); //
        parseStatesInCategory(config["command_states"], "command_states", available_states_, logger); //
        parseStatesInCategory(config["error_states"], "error_states", available_states_, logger); //
        bool idle_found = false; for(const auto& s : available_states_) if(s.id == 0) idle_found = true; //
        if(!idle_found) available_states_.insert(available_states_.begin(), {"Idle (0)", 0, "idle_default_added"}); //
        std::sort(available_states_.begin(), available_states_.end(), [](const auto& a, const auto& b){ return a.id < b.id; }); //
        RCLCPP_INFO(logger, "Loaded %zu states.", available_states_.size()); //
    } catch (const YAML::Exception &e) { 
        RCLCPP_ERROR(logger, "Failed to parse states YAML file '%s': %s", yaml_path.c_str(), e.what()); //
        QMessageBox::critical(this, "YAML Parse Error", QString("Error parsing state file: %1\n%2").arg(QString::fromStdString(yaml_path)).arg(e.what())); //
        return false; 
    }
    return true;
}

std::optional<uint8_t> CommanderPanel::getStateIdByYamlKey(const std::string& yaml_key) const { 
    for (const auto& state_info : available_states_) { if (state_info.yaml_key == yaml_key) return state_info.id; } //
    RVIZ_COMMON_LOG_WARNING_STREAM("CmdPanel: State YAML key '" << yaml_key << "' not found."); //
    return std::nullopt; //
}


// onInitialize, getNode, getLiveRobotNamesFromHeartbeats, publish_kill_states, refresh_robot_list
// onRobotSelectionChanged (for selected_robot_command_publisher_), onFocusCheckBoxToggled, handleUiAfterRobotSelectionChange
// onEnableCurrentButtonClicked, onDisableCurrentButtonClicked, onEnableAllButtonClicked, onDisableAllButtonClicked
// updateButtonAndStatusUI, updateStatusLabel, and utility functions
// remain the same as the previous version where enable/disable use publishers.
// ... (Keep the implementations from the previous correct answer for these methods) ...
void CommanderPanel::onInitialize() {
    auto node = getNode(); if (!node) { RVIZ_COMMON_LOG_ERROR("CmdPanel: Failed to get ROS node."); return; }
    std::string yaml_path_param = "/home/cesar/AMR/src/amr_central/config/command_list.yaml"; 
    try { yaml_path_param = node->declare_parameter<std::string>("robot_states_yaml_path", yaml_path_param); } 
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) { node->get_parameter("robot_states_yaml_path", yaml_path_param); }
    
    RVIZ_COMMON_LOG_INFO_STREAM("CmdPanel: Using states YAML: " << yaml_path_param);
    if (!loadStatesFromYaml(yaml_path_param)) { /* Error message shown in loadStatesFromYaml */ }

    if (uiPanel->stateSelectComboBox) {
        uiPanel->stateSelectComboBox->clear();
        for (const auto& state : available_states_) { uiPanel->stateSelectComboBox->addItem(QString::fromStdString(state.display_name), QVariant(state.id)); }
        uiPanel->stateSelectComboBox->setCurrentIndex(-1);
    }
    try { if (!node->has_parameter("focus_robot")) node->declare_parameter("focus_robot", ""); } 
    catch (const std::exception& e) {RVIZ_COMMON_LOG_ERROR_STREAM("CmdPanel: focus_robot param error: " << e.what());}

    robot_list_timer = node->create_wall_timer(1s, std::bind(&CommanderPanel::refresh_robot_list, this));
    kill_state_publish_timer_ = node->create_wall_timer(1s, std::bind(&CommanderPanel::publish_kill_states, this));

    RVIZ_COMMON_LOG_INFO("CommanderPanel: Initialized.");
}

rclcpp::Node::SharedPtr CommanderPanel::getNode() { 
    auto ctx = getDisplayContext(); if (!ctx) return nullptr;
    auto abs_node = ctx->getRosNodeAbstraction().lock(); return abs_node ? abs_node->get_raw_node() : nullptr;
}

std::vector<std::string> CommanderPanel::getLiveRobotNamesFromHeartbeats() { 
    auto node = getNode(); if (!node) return {}; std::vector<std::string> live_robots;
    auto topics = node->get_topic_names_and_types();
    for (const auto& pair_ : topics) { 
        const std::string& name = pair_.first;
        if (name.find(HEARTBEAT_STRING) != std::string::npos && name.length() > 1 && name[0] == '/') {
            auto components = split_topics_into_componets(name);
            if (!components.empty() && !components[0].empty()) live_robots.push_back(components[0]);
        }
    }
    removeDuplicateStrings(live_robots); return live_robots;
}

void CommanderPanel::publish_kill_states() {
    auto node = getNode();
    if (!node) {
        return;
    }
    std_msgs::msg::UInt8 msg;
    // std::optional<uint8_t> unkilled_id = getStateIdByYamlKey("unkilled"); // Not strictly needed here anymore

    for (auto const& [robot_name, desired_state_id] : robot_desired_kill_states_) {
        auto pub_it = robot_kill_state_publishers_.find(robot_name);
        if (pub_it != robot_kill_state_publishers_.end() && pub_it->second) {
            msg.data = desired_state_id;
            pub_it->second->publish(msg);
        }
    }
}

void CommanderPanel::refresh_robot_list() {
    auto node = getNode(); if (!node) return;
    std::string current_ui_selection = get_current_robot();
    std::string candidate_for_reselection = current_ui_selection; 
    if (uiPanel->focus_box && uiPanel->focus_box->isChecked()) {
        std::string focus_param_val;
        node->get_parameter_or("focus_robot", focus_param_val, std::string(""));
        if (!focus_param_val.empty()) candidate_for_reselection = focus_param_val;
    }
    std::vector<std::string> live_robots = getLiveRobotNamesFromHeartbeats();
    std::optional<uint8_t> unkilled_id = getStateIdByYamlKey("unkilled");
    uint8_t initial_kill_state = unkilled_id.value_or(STATE_CONTINUOUS_NONE); 

    std::unordered_set<std::string> live_robots_set(live_robots.begin(), live_robots.end());
    for (auto it = robot_kill_state_publishers_.begin(); it != robot_kill_state_publishers_.end(); ) {
        if (live_robots_set.find(it->first) == live_robots_set.end()) {
            RVIZ_COMMON_LOG_INFO_STREAM("CmdPanel: Robot " << it->first << " disappeared, removing kill_state publisher.");
            robot_desired_kill_states_.erase(it->first);
            it = robot_kill_state_publishers_.erase(it);
        } else {
            ++it;
        }
    }

    for (const auto& robot_name : live_robots) {
        if (robot_kill_state_publishers_.find(robot_name) == robot_kill_state_publishers_.end()) {
            RVIZ_COMMON_LOG_INFO_STREAM("CmdPanel: New robot " << robot_name << " detected, creating kill_state publisher.");
            try {
                robot_kill_state_publishers_[robot_name] = node->create_publisher<std_msgs::msg::UInt8>("/" + robot_name + "/kill_state", rclcpp::SystemDefaultsQoS());
                if (robot_desired_kill_states_.find(robot_name) == robot_desired_kill_states_.end()) {
                    robot_desired_kill_states_[robot_name] = initial_kill_state;
                }
            } catch (const std::exception& e) {
                 RVIZ_COMMON_LOG_ERROR_STREAM("CmdPanel: Failed to create kill_state publisher for " << robot_name << ": " << e.what());
            }
        }
    }
    
    if (uiPanel->robot_select_block) {
        std::vector<std::string> current_dropdown_items;
        for(int i = 1; i < uiPanel->robot_select_block->count(); ++i) current_dropdown_items.push_back(uiPanel->robot_select_block->itemText(i).toStdString());
        std::sort(current_dropdown_items.begin(), current_dropdown_items.end());
        std::vector<std::string> sorted_live_robots = live_robots; 
        std::sort(sorted_live_robots.begin(), sorted_live_robots.end());

        if (current_dropdown_items != sorted_live_robots) { 
            RVIZ_COMMON_LOG_DEBUG("CmdPanel: Robot list changed, repopulating dropdown.");
            setComboBoxItems(uiPanel->robot_select_block, live_robots, candidate_for_reselection); 
        } else if (!candidate_for_reselection.empty() && candidate_for_reselection != get_current_robot()) {
             int idx = uiPanel->robot_select_block->findText(QString::fromStdString(candidate_for_reselection));
             if (idx > 0) { 
                uiPanel->robot_select_block->blockSignals(true); uiPanel->robot_select_block->setCurrentIndex(idx); uiPanel->robot_select_block->blockSignals(false); 
             } else if (live_robots.empty() && uiPanel->robot_select_block->currentIndex() != 0) { 
                uiPanel->robot_select_block->blockSignals(true); uiPanel->robot_select_block->setCurrentIndex(0); uiPanel->robot_select_block->blockSignals(false); 
             }
        } else if (live_robots.empty() && uiPanel->robot_select_block->currentIndex() != 0) { 
            uiPanel->robot_select_block->blockSignals(true); uiPanel->robot_select_block->setCurrentIndex(0); uiPanel->robot_select_block->blockSignals(false); 
        }
    }
    handleUiAfterRobotSelectionChange(); 
}

void CommanderPanel::onRobotSelectionChanged(int index) {
    (void)index; auto node = getNode(); if(!node) return; 
    std::string newly_selected_robot = get_current_robot(); 
    RVIZ_COMMON_LOG_INFO_STREAM("CmdPanel: UI Robot selection changed to: " << newly_selected_robot); 

    if (!newly_selected_robot.empty()) {
        try {
            // This publisher is for "Enable Current" and "Disable Current"
            selected_robot_command_publisher_ = node->create_publisher<std_msgs::msg::UInt8>("/" + newly_selected_robot + "/command", rclcpp::SystemDefaultsQoS());
            RVIZ_COMMON_LOG_INFO_STREAM("CmdPanel: Command publisher for enable/disable created for /" << newly_selected_robot << "/command");
        } catch (const std::exception& e) {
            RVIZ_COMMON_LOG_ERROR_STREAM("CmdPanel: Failed to create command publisher for " << newly_selected_robot << ": " << e.what());
            selected_robot_command_publisher_ = nullptr;
        }
    } else {
        selected_robot_command_publisher_ = nullptr; 
        RVIZ_COMMON_LOG_INFO("CmdPanel: No robot selected, command publisher cleared.");
    }

    if (uiPanel->focus_box && uiPanel->focus_box->isChecked()) { 
        std::string focus_param_val; node->get_parameter_or("focus_robot", focus_param_val, std::string("")); 
        if (newly_selected_robot != focus_param_val) { 
            RVIZ_COMMON_LOG_INFO_STREAM("CmdPanel: Focus is ON. Updating focus_robot param to: " << newly_selected_robot); 
            try { node->set_parameters({rclcpp::Parameter("focus_robot", newly_selected_robot)}); }  
            catch(const std::exception& e) {RVIZ_COMMON_LOG_ERROR_STREAM("CmdPanel: Failed to set focus_robot param: " << e.what());} 
        }
    }
    handleUiAfterRobotSelectionChange(); 
}

void CommanderPanel::onFocusCheckBoxToggled(bool checked) {
    RVIZ_COMMON_LOG_INFO_STREAM("CmdPanel: Focus checkbox toggled to: " << (checked ? "ON" : "OFF")); 
    refresh_robot_list();  
}

void CommanderPanel::handleUiAfterRobotSelectionChange() {
    std::string current_robot = get_current_robot(); 
    updateButtonAndStatusUI(current_robot); 
}

void CommanderPanel::onEnableCurrentButtonClicked() { 
    std::string current_robot = get_current_robot();
    if (current_robot.empty()) { 
        QMessageBox::warning(this, "No Robot Selected", "Please select a robot first.");
        return; 
    }
    if (!selected_robot_command_publisher_) { // This publisher is for /<robot_name>/command
        QMessageBox::warning(this, "Publisher Error", "Command publisher not available for the selected robot.");
        return;
    }

    std::optional<uint8_t> unkilled_id = getStateIdByYamlKey("unkilled"); 
    if (!unkilled_id) { 
        QMessageBox::critical(this, "Config Error", "'unkilled' state not defined in YAML."); 
        return; 
    }
    
    std_msgs::msg::UInt8 cmd_msg;
    cmd_msg.data = unkilled_id.value();
    selected_robot_command_publisher_->publish(cmd_msg);
    RVIZ_COMMON_LOG_INFO_STREAM("CmdPanel: Published 'Enable' (unkilled: " << (int)cmd_msg.data << ") to /" << current_robot << "/command");

    robot_last_kill_unkill_cmd_[current_robot] = cmd_msg.data;
    robot_commanded_generic_state_.erase(current_robot); 
    updateButtonAndStatusUI(current_robot);
}

void CommanderPanel::onDisableCurrentButtonClicked() { 
    std::string current_robot = get_current_robot();
    if (current_robot.empty()) { 
        QMessageBox::warning(this, "No Robot Selected", "Please select a robot first.");
        return; 
    }
    if (!selected_robot_command_publisher_) { // This publisher is for /<robot_name>/command
        QMessageBox::warning(this, "Publisher Error", "Command publisher not available for the selected robot.");
        return;
    }
    
    std::optional<uint8_t> killed_id = getStateIdByYamlKey("killed");
    if (!killed_id) { 
        QMessageBox::critical(this, "Config Error", "'killed' state not defined in YAML."); 
        return; 
    }

    std_msgs::msg::UInt8 cmd_msg;
    cmd_msg.data = killed_id.value();
    selected_robot_command_publisher_->publish(cmd_msg);
    RVIZ_COMMON_LOG_INFO_STREAM("CmdPanel: Published 'Disable' (killed: " << (int)cmd_msg.data << ") to /" << current_robot << "/command");
    
    robot_last_kill_unkill_cmd_[current_robot] = cmd_msg.data;
    robot_commanded_generic_state_.erase(current_robot);
    updateButtonAndStatusUI(current_robot);
}

void CommanderPanel::onEnableAllButtonClicked() {
    std::optional<uint8_t> unkilled_id = getStateIdByYamlKey("unkilled"); 
    if (!unkilled_id) {  
        RVIZ_COMMON_LOG_ERROR("CmdPanel: 'unkilled' YAML key not found for EnableAll.");  
        QMessageBox::warning(this, "State Error", "Unkilled state ID not defined in YAML config.");  
        return;  
    }
    
    std::string cmd_desc = "Unkill All";  
    for(const auto& s_info : available_states_) if(s_info.id == unkilled_id.value()) cmd_desc = s_info.display_name; 
    RVIZ_COMMON_LOG_INFO_STREAM("CmdPanel: 'Enable All Robots' setting desired state to " << cmd_desc << " for all live robots."); 
    
    std::vector<std::string> live_robots = getLiveRobotNamesFromHeartbeats();  
    if (live_robots.empty()) { 
        QMessageBox::information(this, "No Live Robots", "No live robots to command."); return; 
    }
    int success_count = 0; 
    for (const auto& robot_name : live_robots) { 
        robot_desired_kill_states_[robot_name] = unkilled_id.value();
        robot_last_kill_unkill_cmd_[robot_name] = unkilled_id.value(); 
        robot_commanded_generic_state_.erase(robot_name);
        success_count++; 
    }
    RVIZ_COMMON_LOG_INFO_STREAM("CmdPanel: Set desired state to " << cmd_desc << " for " << success_count << " live robots. Will be published via /kill_state topics."); 
    updateButtonAndStatusUI(get_current_robot()); 
}

void CommanderPanel::onDisableAllButtonClicked() {
    std::optional<uint8_t> killed_id = getStateIdByYamlKey("killed"); 
    if (!killed_id) {  
        RVIZ_COMMON_LOG_ERROR("CmdPanel: 'killed' YAML key not found for DisableAll.");  
        QMessageBox::warning(this, "State Error", "Killed state ID not defined in YAML config.");  
        return;  
    }

    std::string cmd_desc = "Kill All"; 
    for(const auto& s_info : available_states_) if(s_info.id == killed_id.value()) cmd_desc = s_info.display_name; 
    RVIZ_COMMON_LOG_INFO_STREAM("CmdPanel: 'Disable All Robots' setting desired state to " << cmd_desc << " for all live robots."); 

    std::vector<std::string> live_robots = getLiveRobotNamesFromHeartbeats(); 
    if (live_robots.empty()) { 
        QMessageBox::information(this, "No Live Robots", "No live robots to command."); return; 
    }
    int success_count = 0; 
    for (const auto& robot_name : live_robots) { 
        robot_desired_kill_states_[robot_name] = killed_id.value();
        robot_last_kill_unkill_cmd_[robot_name] = killed_id.value(); 
        robot_commanded_generic_state_.erase(robot_name);
        success_count++; 
    }
    RVIZ_COMMON_LOG_INFO_STREAM("CmdPanel: Set desired state to " << cmd_desc << " for " << success_count << " live robots. Will be published via /kill_state topics."); 
    updateButtonAndStatusUI(get_current_robot()); 
}

// THIS IS THE MODIFIED METHOD
void CommanderPanel::onSetStateButtonClicked() {
    std::string current_robot = get_current_robot();
    if (current_robot.empty()) { 
        QMessageBox::warning(this, "No Robot Selected", "Select a robot to set state."); 
        return; 
    }
    if (!uiPanel || !uiPanel->stateSelectComboBox || uiPanel->stateSelectComboBox->currentIndex() < 0) { 
        QMessageBox::warning(this, "No State Selected", "Select a state from the dropdown."); 
        return; 
    }

    auto node = getNode();
    if (!node) {
        RVIZ_COMMON_LOG_ERROR("CmdPanel: No node for general service call (Set State).");
        QMessageBox::critical(this, "ROS Error", "Cannot get ROS node to send command.");
        return;
    }

    QVariant data = uiPanel->stateSelectComboBox->currentData(); 
    bool ok;
    uint8_t state_id = static_cast<uint8_t>(data.toUInt(&ok));
    if (!ok) { 
        QMessageBox::warning(this, "State Error", "Invalid state data in dropdown selection."); 
        return; 
    }

    std::string service_name = "/" + current_robot + ROBOT_STATE_CHANGE_SERVICE_SUFFIX;
    auto client = node->create_client<amr_msgs::srv::StateChangeRequest>(service_name);

    if (!client) {
        RVIZ_COMMON_LOG_ERROR_STREAM("CmdPanel: Failed to create service client for " << service_name);
        QMessageBox::critical(this, "ROS Error", QString("Failed to create service client for %1").arg(QString::fromStdString(service_name)));
        return;
    }
    
    // Optional: Add a timeout for waiting for the service
    // if (!client->wait_for_service(std::chrono::seconds(1))) {
    //     RVIZ_COMMON_LOG_WARNING_STREAM("CmdPanel: Service " << service_name << " not available.");
    //     QMessageBox::warning(this, "Service Error", QString("Service %1 not available.").arg(QString::fromStdString(service_name)));
    //     return;
    // }

    auto request = std::make_shared<amr_msgs::srv::StateChangeRequest::Request>();
    request->requested_states = {state_id};
    
    std::string context_str = "PanelGeneralCmd";
    std::string state_display_name = "ID " + std::to_string(state_id);
    for(const auto& s_info : available_states_) { 
        if(s_info.id == state_id) { 
            context_str = "PanelCmd:" + s_info.yaml_key; 
            state_display_name = s_info.display_name;
            break; 
        }
    }
    request->context = context_str;

    RVIZ_COMMON_LOG_INFO_STREAM("CmdPanel: Calling service " << service_name << " for robot " << current_robot << " to set state " << state_display_name);
    
    client->async_send_request(request, 
        [this, service_name, state_id, current_robot, state_display_name](rclcpp::Client<amr_msgs::srv::StateChangeRequest>::SharedFuture future) {
        try {
            auto response = future.get(); 
            RVIZ_COMMON_LOG_INFO_STREAM("CmdPanel: Service " << service_name << " for state " << state_display_name 
                                        << " (ID " << (int)state_id << ") acked. Robot accepted state: " << (int)response->accepted_state);
            // Update UI based on the actual accepted state
            robot_commanded_generic_state_[current_robot] = response->accepted_state; 
            robot_last_kill_unkill_cmd_.erase(current_robot); 
            // QTimer ensures UI update happens in the main thread
            QTimer::singleShot(0, this, [this, robot_name = current_robot](){ this->updateButtonAndStatusUI(robot_name); });
        } catch (const std::exception &e) { 
            RVIZ_COMMON_LOG_WARNING_STREAM("CmdPanel: Service call " << service_name << " for state " << state_display_name << " failed: " << e.what());
             QTimer::singleShot(0, this, [service_name_str = service_name, e_what = std::string(e.what())](){ 
                QMessageBox::warning(nullptr, "Service Call Failed", QString("Service %1 failed: %2").arg(QString::fromStdString(service_name_str)).arg(QString::fromStdString(e_what)));
            });
        }
    });

    // Optimistically update UI, will be corrected by callback if needed
    robot_commanded_generic_state_[current_robot] = state_id; 
    robot_last_kill_unkill_cmd_.erase(current_robot); 
    updateButtonAndStatusUI(current_robot); 
}


void CommanderPanel::updateButtonAndStatusUI(const std::string& robot_name) {
    bool robot_selected = !robot_name.empty();
    if(uiPanel->enableCurrentButton) uiPanel->enableCurrentButton->setEnabled(robot_selected);
    if(uiPanel->disableCurrentButton) uiPanel->disableCurrentButton->setEnabled(robot_selected);
    if(uiPanel->setStateButton) uiPanel->setStateButton->setEnabled(robot_selected);
    if(uiPanel->stateSelectComboBox) uiPanel->stateSelectComboBox->setEnabled(robot_selected);

    uint8_t label_state_code = STATE_ERROR_UNKNOWN; 
    bool state_known_for_label = false; 

    if (!robot_name.empty()) { 
        auto kill_it = robot_last_kill_unkill_cmd_.find(robot_name); 
        if (kill_it != robot_last_kill_unkill_cmd_.end()) {  
            label_state_code = kill_it->second; state_known_for_label = true;  
        } else { 
            auto generic_it = robot_commanded_generic_state_.find(robot_name);  
            if (generic_it != robot_commanded_generic_state_.end()) {  
                label_state_code = generic_it->second; state_known_for_label = true;  
            } else {
                auto desired_kill_it = robot_desired_kill_states_.find(robot_name);
                if (desired_kill_it != robot_desired_kill_states_.end()){
                    label_state_code = desired_kill_it->second; state_known_for_label = true;
                }
            }
        }
    }
    updateStatusLabel(robot_name, state_known_for_label ? label_state_code : STATE_ERROR_UNKNOWN); 
}

void CommanderPanel::updateStatusLabel(const std::string& robot_name, uint8_t state_code) { 
    if (!uiPanel->currentRobotStatusLabel) return; 
    if (robot_name.empty()) { uiPanel->currentRobotStatusLabel->setText("Status: No Robot Selected"); }  
    else {
        QString status_text = QString("Robot '%1' Status: ").arg(QString::fromStdString(robot_name)); 
        bool found = false; for(const auto& s_info : available_states_) { if (s_info.id == state_code) { status_text += QString::fromStdString(s_info.display_name); found = true; break; } } 
        if (!found) status_text += QString("Code %1 (Unknown)").arg(state_code); 
        uiPanel->currentRobotStatusLabel->setText(status_text); 
    }
}

std::vector<std::string> CommanderPanel::split_topics_into_componets(const std::string& str) { 
    std::vector<std::string> tokens; std::string t;  
    if (str.empty() || str[0] != '/') { return tokens; } 
    for (size_t i = 1; i < str.length(); ++i) {  
        if (str[i] == '/') { if(!t.empty()) { tokens.push_back(t); } t.clear(); }  
        else { t+=str[i]; } 
    }
    if(!t.empty()) { tokens.push_back(t); }  
    return tokens; 
}

void CommanderPanel::removeDuplicateStrings(std::vector<std::string>& vec) { 
    std::vector<std::string> res; std::unordered_set<std::string> s; 
    for(const auto& i : vec) if(!i.empty() && s.find(i)==s.end()) {s.insert(i); res.push_back(i);} 
    std::sort(res.begin(), res.end()); vec = res; 
}

void CommanderPanel::setComboBoxItems(QComboBox* comboBox, const std::vector<std::string>& items, const std::string& desired_selection) {
    if (!comboBox) { return; }  
    comboBox->blockSignals(true);  
    comboBox->clear();  
    comboBox->addItem("");  
    int selection_idx = 0;  
    for (size_t i = 0; i < items.size(); ++i) { 
        if (!items[i].empty()) {  
            comboBox->addItem(QString::fromStdString(items[i]));  
            if (items[i] == desired_selection) {  
                selection_idx = i + 1;  
            }
        }
    }
    if (selection_idx == 0 && comboBox->count() > 1 ) {  
        bool desired_was_live_and_is_now_gone = false; 
        if(!desired_selection.empty()){ 
            bool still_live = false; 
            for(const auto& item : items) if(item == desired_selection) still_live = true; 
            if(!still_live) desired_was_live_and_is_now_gone = true; 
        }
        
        if(desired_was_live_and_is_now_gone || (desired_selection.empty() && !items.empty())) { 
            selection_idx = 1; 
        }
    } else if (comboBox->count() == 1) {  
        selection_idx = 0; 
    }
    comboBox->setCurrentIndex(selection_idx);  
    comboBox->blockSignals(false); 
}

std::string CommanderPanel::get_current_robot() { 
    if (uiPanel && uiPanel->robot_select_block && uiPanel->robot_select_block->currentIndex() > 0) { return uiPanel->robot_select_block->currentText().toStdString(); }  
    return ""; 
}

void CommanderPanel::load(const rviz_common::Config &config) { 
   rviz_common::Panel::load(config); int count = 0; 
   config.mapGetInt("RobotGenericStateCount", &count);  
   if (count > 0) { robot_commanded_generic_state_.clear();  
       for (int i=0; i<count; ++i) { QString n; int s=0; if (config.mapGetString(QString("RGS_N%1").arg(i),&n) && config.mapGetInt(QString("RGS_S%1").arg(i),&s)) robot_commanded_generic_state_[n.toStdString()] = (uint8_t)s;}} 
   config.mapGetInt("RobotKillStateCount", &count); 
   if (count > 0) { robot_last_kill_unkill_cmd_.clear(); 
       for (int i=0; i<count; ++i) { QString n; int s=0; if (config.mapGetString(QString("RKS_N%1").arg(i),&n) && config.mapGetInt(QString("RKS_S%1").arg(i),&s)) robot_last_kill_unkill_cmd_[n.toStdString()] = (uint8_t)s;}} 
   QTimer::singleShot(200, this, [this](){ this->refresh_robot_list(); }); 
}

void CommanderPanel::save(rviz_common::Config config) const { 
    rviz_common::Panel::save(config); 
    config.mapSetValue("RobotGenericStateCount", (int)robot_commanded_generic_state_.size()); int i=0; 
    for (const auto& p : robot_commanded_generic_state_) { config.mapSetValue(QString("RGS_N%1").arg(i), QString::fromStdString(p.first)); config.mapSetValue(QString("RGS_S%1").arg(i), (int)p.second); i++;} 
    config.mapSetValue("RobotKillStateCount", (int)robot_last_kill_unkill_cmd_.size()); i=0; 
    for (const auto& p : robot_last_kill_unkill_cmd_) { config.mapSetValue(QString("RKS_N%1").arg(i), QString::fromStdString(p.first)); config.mapSetValue(QString("RKS_S%1").arg(i), (int)p.second); i++;} 
}

double CommanderPanel::get_ros2_time() { 
    auto n = getNode(); return n ? n->get_clock()->now().seconds() : 0.0;  
}


} // namespace amrviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(amrviz::CommanderPanel, rviz_common::Panel)