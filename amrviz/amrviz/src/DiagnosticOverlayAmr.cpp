#include "amrviz/DiagnosticOverlayAmr.hpp"

#include <QFontDatabase>
#include <QMessageBox>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

#include <boost/algorithm/string.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

#define ROBOT_HEARTBEAT_REFRESH_TIME 10

namespace amrviz
{
    DiagnosticOverlayAmr::DiagnosticOverlayAmr(){
        // init font parameter
        QFontDatabase database;
        fontFamilies = database.families();
        fontProperty = new rviz_common::properties::EnumProperty("font", "DejaVu Sans Mono", "font", this, SLOT(updateFont()));
        for (ssize_t i = 0; i < fontFamilies.size(); i++) {
            fontProperty->addOption(fontFamilies[i], (int) i);
        }

        timeoutProperty = new rviz_common::properties::FloatProperty(
            "diagnostic_timeout", 10.0, "Maximum time between diagnostic packets before default", this);


    }

    DiagnosticOverlayAmr::~DiagnosticOverlayAmr(){
        delete robotNsProperty;
        delete timeoutProperty;
    }

    void DiagnosticOverlayAmr::onInitialize(){

        OverlayDisplay::onInitialize();

        // get our local rosnode
        auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

        // backdate timeouts
        lastDiag = node->get_clock()->now() - 1h;
        lastKill = node->get_clock()->now() - 1h;
        // lastZed = node->get_clock()->now() - 1h;
        // lastLeak = node->get_clock()->now() - 1h;

        // make the diagnostic subscriber
        diagSub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics_agg", rclcpp::SystemDefaultsQoS(), std::bind(&DiagnosticOverlayAmr::diagnosticCallback, this, _1)
        );

        // std::string zedTopic = robotNsProperty->getStdString() + "/zed/zed_node/temperature/imu";
        // zedSub = node->create_subscription<sensor_msgs::msg::Temperature>(
        //     zedTopic, rclcpp::SystemDefaultsQoS(), std::bind(&DiagnosticOverlayAmr::zedCallback, this, _1)
        // );

        // std::string leakTopic = robotNsProperty->getStdString() + "/state/leak";
        // leakSub = node->create_subscription<std_msgs::msg::Bool>(
        //     leakTopic, rclcpp::SystemDefaultsQoS(), std::bind(&DiagnosticOverlayAmr::leakCallback, this, _1)
        // );

        // // watchdog timers for handling timeouts
        // checkTimer = node->create_wall_timer(0.25s, std::bind(&DiagnosticOverlayAmr::checkTimeout, this));

        //timer for checking for available robots
        availableRobotRefreshTimer = node->create_wall_timer(5s, std::bind(&DiagnosticOverlayAmr::refreshAvailableRobots, this));

        //timer for refreshing robot statuses
        detectedRobotsStatusTimer = node->create_wall_timer(1s, std::bind(&DiagnosticOverlayAmr::RobotStatusCheckCallback, this));

        // add all of the variable design items
        // voltageConfig.text_color_ = QColor(255, 0, 255, 255);
        // voltageTextId = addText(voltageConfig);

        diagLedConfig.inner_color_ = QColor(255, 0, 255, 255);
        diagLedConfigId = addCircle(diagLedConfig);

        killLedConfig.inner_color_ = QColor(255, 0, 255, 255);
        killLedConfigId = addCircle(killLedConfig);

        // zedLedConfig.inner_color_ = QColor(255, 0, 255, 255);
        // zedLedConfigId = addCircle(zedLedConfig);

        // leakLedConfig.inner_color_ = QColor(255, 0, 255, 255);
        // leakLedConfigId = addCircle(leakLedConfig);
        
        //int id = addCircle(zedLedConfig);

        //updateCircle(id, zedLedConfig);

        // add the static design items
        riptide_rviz::PaintedTextConfig diagLedLabel = {
            6, 20, 0, 0, "Diag",
            fontName, false, 2, 12,
            QColor(255, 255, 255, 255)
        };
        riptide_rviz::PaintedTextConfig killLedLabel = {
            50, 20, 0, 0, "Kill",
            fontName, false, 2, 12,
            QColor(255, 255, 255, 255)
        };

        // riptide_rviz::PaintedTextConfig zedLedLabel = {
        //     87, 20, 0, 0, "Zed",
        //     fontName, false, 2, 12,
        //     QColor(255, 255, 255, 255)
        // };
        // riptide_rviz::PaintedTextConfig leakLedLabel = {
        //     6, 70, 0, 0, "Leak",
        //     fontName, false, 2, 12,
        //     QColor(255, 255, 255, 255)
        // };
        addText(diagLedLabel);
        addText(killLedLabel);
        // addText(zedLedLabel);
        // addText(leakLedLabel);

    }

    void DiagnosticOverlayAmr::diagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray & msg){
        // get our local rosnode
        auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

        // write down the timestamp that it was recieved
        lastDiag = node->get_clock()->now();
        diagsTimedOut = false;

        // look for specific packets
        for(auto diagnostic : msg.status){
            // handle robot voltage packet
            // if(diagnostic.name == "/Robot Diagnostics/Electronics/Voltages and Currents/V+ Rail Voltage"){
            //     bool found = false;
            //     voltageConfig.text_ = "00.00 V";
            //     voltageConfig.text_color_ = QColor(255, 0, 0, 255);
                
            //     for(auto pair : diagnostic.values){
            //         if(pair.key == "V+ Rail Voltage"){
            //             found = true;
            //             voltageConfig.text_ = pair.value;
            //         }
            //     }

            //     if(!found){
            //         voltageConfig.text_ = "BAD CONV";
            //     }

            //     // now we need to look at the status of the voltage to determine color
            //     // ok is green, warn is yellow, error is red
            //     if(diagnostic.level == diagnostic.ERROR){
            //         voltageConfig.text_color_ = QColor(255, 0, 0, 255);
            //     } else if (diagnostic.level == diagnostic.WARN){
            //         voltageConfig.text_color_ = QColor(255, 255, 0, 255);
            //     } else if (diagnostic.level == diagnostic.STALE){
            //         diagLedConfig.inner_color_ = QColor(255, 0, 255, 255);
            //     } else {
            //         voltageConfig.text_color_ = QColor(0, 255, 0, 255);
            //     }
                

            //     // edit the text
            //     updateText(voltageTextId, voltageConfig);
            // }

            // handle general packet for LED
            if(diagnostic.name == "/Robot Diagnostics"){
                // Determine the LED color to use
                if(diagnostic.level == diagnostic.ERROR){
                    diagLedConfig.inner_color_ = QColor(255, 0, 0, 255);
                } else if (diagnostic.level == diagnostic.WARN){
                    diagLedConfig.inner_color_ = QColor(255, 255, 0, 255);
                } else if (diagnostic.level == diagnostic.STALE){
                    diagLedConfig.inner_color_ = QColor(255, 0, 255, 255);
                } else {
                    diagLedConfig.inner_color_ = QColor(0, 255, 0, 255);
                }

                updateCircle(diagLedConfigId, diagLedConfig);
            }
        }
    }

    // void DiagnosticOverlayAmr::zedCallback(const sensor_msgs::msg::Temperature& msg) {
    //     // get our local rosnode
    //     auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

    //     zedTimedOut = false;

    //     zedLedConfig.inner_color_ = QColor(0, 255, 0, 255);
    //     updateCircle(zedLedConfigId, zedLedConfig);

    //     lastZed = node->get_clock()->now();
    // }

    // void DiagnosticOverlayAmr::leakCallback(const std_msgs::msg::Bool& msg) {
    //     static bool redFlash = true;

    //     // get our local rosnode
    //     auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

    //     leakTimedOut = false;

    //     if (msg.data) {
    //         if (redFlash) {
    //             leakLedConfig.inner_color_ = QColor(255, 0, 0, 255);  // Flash red
    //             redFlash = false;
    //         }
    //         else {
    //             leakLedConfig.inner_color_ = QColor(252, 126, 0, 255);  // Flash orange
    //             redFlash = true;
    //         }

    //         if (!startedLeaking) {
    //             // Pop up an annoying box
    //             QMessageBox msgBox;
    //             msgBox.setWindowTitle("LEAK");
    //             msgBox.setIcon(QMessageBox::Warning);
    //             msgBox.setText(QString::fromStdString("Water was detected in one of the cages."));

    //             msgBox.setStandardButtons(QMessageBox::Ok);
    //             msgBox.setDefaultButton(QMessageBox::Ok);
    //             msgBox.exec();  
                
    //             startedLeaking = true;
    //         }

    //     }
    //     else {
    //         leakLedConfig.inner_color_ = QColor(0, 255, 0, 255);
    //         startedLeaking = false;
    //     }

    //     updateCircle(leakLedConfigId, leakLedConfig);
    //     lastLeak = node->get_clock()->now();
    // }

    void DiagnosticOverlayAmr::killCallback(const std_msgs::msg::Bool & msg){
        // get our local rosnode
        auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

        // write down the time that we recieve the last kill msg
        lastKill = node->get_clock()->now();
        killTimedOut = false;

        if(!msg.data){
            killLedConfig.inner_color_ = QColor(0, 255, 0, 255);
        } else {
            killLedConfig.inner_color_ = QColor(255, 0, 0, 255);
        }
        updateCircle(killLedConfigId, killLedConfig);
    }

    void DiagnosticOverlayAmr::updateFont() {
        int font_index = fontProperty->getOptionInt();
        if (font_index < fontFamilies.size()) {
            fontName = fontFamilies[font_index].toStdString();
        } else {
            RVIZ_COMMON_LOG_ERROR_STREAM("DiagnosticOverlay: Unexpected error at selecting font index " << font_index);
            return;
        }


        require_update_texture_ = true;
    }
    
    void DiagnosticOverlayAmr::onEnable(){
        OverlayDisplay::onEnable();
    }
    
    void DiagnosticOverlayAmr::onDisable(){
        OverlayDisplay::onDisable();
    }
    
    void DiagnosticOverlayAmr::update(float wall_dt, float ros_dt){
        OverlayDisplay::update(wall_dt, ros_dt);
    }

    // void DiagnosticOverlayAmr::checkTimeout(){
    //     // read current timeout property and convert to duration
    //     auto timeoutDur = std::chrono::duration<double>(timeoutProperty->getFloat());

    //     // get our local rosnode
    //     auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

    //     // check for diagnostic timeout
    //     auto duration = node->get_clock()->now() - lastDiag;
    //     if(duration > timeoutDur){
    //         if(! diagsTimedOut){
    //             RVIZ_COMMON_LOG_WARNING("DiagnosticsOverlay: Diagnostics timed out!");
    //             diagsTimedOut = true;
    //         }

    //         // diagnostics timed out, reset them
    //         voltageConfig.text_color_ = QColor(255, 0, 255, 255);
    //         updateText(voltageTextId, voltageConfig);

    //         diagLedConfig.inner_color_ = QColor(255, 0, 255, 255);
    //         updateCircle(diagLedConfigId, diagLedConfig);
    //     }

    //     // check for kill timeout
    //     duration = node->get_clock()->now() - lastKill;
    //     if(duration > timeoutDur){
    //         if(! killTimedOut){
    //             RVIZ_COMMON_LOG_WARNING("DiagnosticsOverlay: Kill switch timed out!");
    //             killTimedOut = true;
    //         }

    //         // diagnostics timed out, reset them
    //         killLedConfig.inner_color_ = QColor(255, 0, 255, 255);
    //         updateCircle(killLedConfigId, killLedConfig);
    //     }


    //     duration = node->get_clock()->now() - lastZed;
    //     if (duration > std::chrono::duration<double>(1.0f)) {
    //         if (!zedTimedOut) {
    //             RVIZ_COMMON_LOG_WARNING("DiagnosticsOverlay: Zed connection timed out!");
    //             zedTimedOut = true;
    //         }

    //         zedLedConfig.inner_color_ = QColor(255, 0, 0, 255);
    //         updateCircle(zedLedConfigId, zedLedConfig);
    //     }

    //     duration = node->get_clock()->now() - lastLeak;
    //     if (duration > std::chrono::duration<double>(2.0s)) {
    //         if (!leakTimedOut) {
    //             RVIZ_COMMON_LOG_WARNING("DiagnosticsOverlay: Leak sensors timed out!");
    //             leakTimedOut = true;
    //         }

    //         leakLedConfig.inner_color_ = QColor(255, 0, 255, 255);
    //         updateCircle(leakLedConfigId, leakLedConfig);
    //     }
    // }

    void DiagnosticOverlayAmr::refreshAvailableRobots(){

        // get ros node
        auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

        //get topic map
        std::map<std::string, std::vector<std::string>> topics = node->get_topic_names_and_types();
        
        //look for new robots in topic map
        std::map<std::string, std::vector<std::string>>::iterator itr;
        for(itr = topics.begin(); itr != topics.end(); itr++){

            //if the topic is a heartbeat
            if(itr->second.size() > 0){
                if(!itr->second.front().compare("amr_msgs/msg/Heartbeat")){

                    std::vector<std::string> parts;
                    boost::split(parts, itr->first, boost::is_any_of("/"));

                    //check to see if the robot has alread been detected
                    bool detected = false;
                    std::map<std::string, std::pair<double, std::pair<int, std::pair<int, int>>>>::iterator inner_itr;
                    for(inner_itr = detected_robots.begin(); inner_itr != detected_robots.end(); inner_itr++){
                        if(!inner_itr->first.compare(parts.at(1))){
                            detected = true;
                        }
                    }

                    //if not in list add 
                    if(!detected){
                        //current time
                        double current_time = node->get_clock()->now().seconds();

                        // x of lights
                        int x_pos = 110;
                        if(robot_diag_lights.size() > 0){
                            x_pos = robot_diag_lights[robot_diag_lights.size() - 1].x_ + 50;
                        }                        
                        
                        //create display config
                        riptide_rviz::PaintedCircleConfig circle_pair_config = {
                            x_pos, 50, 0, 0, 7, 9,
                            QColor(255, 0, 255, 255),
                            QColor(0, 0, 0, 255)
                        };

                        circle_pair_config.inner_color_ = QColor(255, 255, 0, 255);

                        //add display config to 
                        robot_diag_lights.push_back(circle_pair_config);
                        int led_id = addCircle(robot_diag_lights[robot_diag_lights.size() - 1]);

                        //add voltage label
                        riptide_rviz::PaintedTextConfig robot_voltage_label = {
                            x_pos - 20, 0, 40, 0, "00.00V",
                            fontName, false, 2, 12,
                            QColor(255, 255, 255, 255)
                        };
                        robot_voltage_labels.push_back(robot_voltage_label);
                        int voltage_id = addText(robot_voltage_labels[robot_voltage_labels.size() - 1]);
                        
                        //add to list to check in the future
                        std::pair<int, int> id_pair = std::pair<int, int>(voltage_id, led_id);
                        std::pair<int, std::pair<int, int>> circlepair = std::pair<int, std::pair<int, int>>(robot_diag_lights.size() - 1, id_pair);
                        std::pair<double, std::pair<int, std::pair<int, int>>> value_pair = std::pair<double, std::pair<int, std::pair<int, int>>>(current_time, circlepair);
                        detected_robots.insert(std::pair<std::string, std::pair<double, std::pair<int, std::pair<int, int>>>>(parts.at(1), value_pair));

                        //add text to the dot
                        riptide_rviz::PaintedTextConfig robot_diag_label = {
                            x_pos - 24, 21, 48, 0, parts.at(1),
                            fontName, false, 2, 10,
                            QColor(255, 255, 255, 255)
                        };
                        addText(robot_diag_label);

                        //create callback
                        diag_subs.push_back(node->create_subscription<amr_msgs::msg::Heartbeat>(itr->first, rclcpp::SystemDefaultsQoS(), std::bind(&DiagnosticOverlayAmr::RobotHeartbeatCallback, this, _1)));

                        //ensure their is enought width for all lights
                        setWidth(x_pos + 50);
                    }
                }
            }
        }
    }

    void DiagnosticOverlayAmr::RobotStatusCheckCallback(){
        // get ros node
        auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

        std::map<std::string, std::pair<double, std::pair<int, std::pair<int, int>>>>::iterator it;

        double current_time = node->get_clock()->now().seconds();

        for(it = detected_robots.begin(); it != detected_robots.end(); it++){
            if(!(it->second.first + ROBOT_HEARTBEAT_REFRESH_TIME > current_time)){
                //give lost status light
                robot_diag_lights[it->second.second.first].inner_color_ =  QColor(255, 0, 255, 255);
                updateCircle(it->second.second.second.second, robot_diag_lights[it->second.second.first]);            
            }

            RVIZ_COMMON_LOG_INFO_STREAM("Here");
        }
    }

    void DiagnosticOverlayAmr::RobotHeartbeatCallback(const amr_msgs::msg::Heartbeat & msg){
        // get ros node
        auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

        //refresh the time the robot was detected
        detected_robots[msg.robot_name].first = node->get_clock()->now().seconds();

        //check for any sensor disconnections -> if so set light to orange
        if(!msg.left_esc_connected || !msg.right_esc_connected || !msg.left_ir_connected || !msg.right_ir_connected || !msg.uid_connected){
            //orange error light
            robot_diag_lights[detected_robots[msg.robot_name].second.first].inner_color_ = QColor(255, 165, 0, 255);
            updateCircle(detected_robots[msg.robot_name].second.second.second, robot_diag_lights[detected_robots[msg.robot_name].second.first]);
        } else {
            //give positive status light
            robot_diag_lights[detected_robots[msg.robot_name].second.first].inner_color_ = QColor(0, 255, 0, 255);
            updateCircle(detected_robots[msg.robot_name].second.second.second, robot_diag_lights[detected_robots[msg.robot_name].second.first]);
        }

        //update the voltage display
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << msg.battery_voltage;
        std::string voltage_string = ss.str() + "V";
        robot_voltage_labels[detected_robots[msg.robot_name].second.first].text_ = voltage_string;
        updateText(detected_robots[msg.robot_name].second.second.first, robot_voltage_labels[detected_robots[msg.robot_name].second.first]);

    } 

    void DiagnosticOverlayAmr::reset(){
        OverlayDisplay::reset();
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(amrviz::DiagnosticOverlayAmr, rviz_common::Display)