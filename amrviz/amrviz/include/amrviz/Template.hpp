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
#include "ui_template.h"
#include <string>
#include <stdio.h>
#include <vector>


#define HEARTBEAT_STRING "heartbeat"

using namespace std::chrono_literals;

namespace amrviz
{

    class Template : public rviz_common::Panel
    {
        Q_OBJECT public : Template(QWidget *parent = 0);
        ~Template();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
        void onInitialize() override;

        //timer callbacks
        void refresh_robot_list();
        void update_robot_based_subscribers();

        //subscriber callbacks

       
        // UI Panel instance
        Ui_Template *uiPanel;

        //ros stuff


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
