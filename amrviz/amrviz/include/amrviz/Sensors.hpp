#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <QTimer>
#include <QMessageBox>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include "ui_sensors.h"
#include <string>
#include <stdio.h>
#include <vector>

#define HEARTBEAT_STRING "heartbeat"
#define LEFT_IR_TOPIC "/state/ir/left"
#define RIGHT_IR_TOPIC "/state/ir/right"
#define LEFT_ENCODER_TOPIC "/state/encoder/left"
#define RIGHT_ENCODER_TOPIC "/state/encoder/right"
#define RFID_TOPIC "/state/rfid"

using namespace std::chrono_literals;

namespace amrviz
{

    class Sensors : public rviz_common::Panel
    {
        Q_OBJECT public : Sensors(QWidget *parent = 0);
        ~Sensors();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
        void onInitialize() override;

        //timer callbacks
        void refresh_robot_list();
        void update_robot_based_subscribers();

        //subscriber callbacks
        void RFID_update_cb(const std_msgs::msg::String& msg);
        void IR_left_update_cb(const std_msgs::msg::Float32& msg);
        void IR_right_update_cb(const std_msgs::msg::Float32& msg);
        void encoder_left_update_cb(const std_msgs::msg::Float32& msg);
        void encoder_right_update_cb(const std_msgs::msg::Float32& msg);

       
        // UI Panel instance
        Ui_Sensors *uiPanel;

        //ros stuff
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr IR_sub_L;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr IR_sub_R;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encoder_sub_L;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encoder_sub_R;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr RFID_sub;

        //robot refresh timer
        rclcpp::TimerBase::SharedPtr robot_list_timer;
        rclcpp::TimerBase::SharedPtr robot_sensor_update_timer;

        //helpers
        std::vector<std::string> split_topics_into_componets(const std::string& str);
        void setComboBoxItems(QComboBox* comboBox, const std::vector<std::string>& items);
        void removeDuplicateStrings(std::vector<std::string>& vec);
        double get_ros2_time();
        std::string get_current_robot();
    };

} // namespace riptide_rviz
