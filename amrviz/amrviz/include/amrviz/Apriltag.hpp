#pragma once

#include "ui_apriltag.h"

#include <string>
#include <stdio.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <QTimer>
#include <QWidget>
#include <QMessageBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QApplication>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"


#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define MAP_FRAME "map"

#define HEARTBEAT_STRING "heartbeat"

#define TAG_STALE_PARAM_NAME "tag_transform_stale_time"

#define TAG_LIST_SUBPATH "/test_data/april_tag_lookup.yaml"

using namespace std::chrono_literals;

namespace amrviz
{

    class Apriltag : public rviz_common::Panel
    {
        Q_OBJECT public : Apriltag(QWidget *parent = 0);
        ~Apriltag();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
        void onInitialize() override;

        void load_in_tags();

        //timer callbacks
        void refresh_robot_list();
        void update_robot_based_subscribers();
        void repub_tags();

        //subscriber callbacks

       
        // UI Panel instance
        Ui_Apriltag *uiPanel;

        //ros stuff


        //robot refresh timer
        rclcpp::TimerBase::SharedPtr robot_list_timer;
        rclcpp::TimerBase::SharedPtr robot_sensor_update_timer;
        
        //other timers
        rclcpp::TimerBase::SharedPtr update_published_tags_timer;

        //helpers
        std::vector<std::string> split_topics_into_componets(const std::string& str);
        void setComboBoxItems(QComboBox* comboBox, const std::vector<std::string>& items);
        void removeDuplicateStrings(std::vector<std::string>& vec);
        double get_ros2_time();
        int getClassTagIndex(const std::string& new_tag, const std::vector<std::string>& classes, std::string* class_name);
        int get_id_from_tag_string(const std::string& tag_string);
        std::vector<int> filter_available_tags(std::vector<int> tags_to_filter);

        //combo getters
        std::string get_current_robot();
        std::string get_selected_tag_type();
        std::string get_selected_tag();
        std::string get_remove_tag();

        //text box getters
        double get_tag_x_position();
        double get_tag_y_position();

        //tag lists
        std::vector<std::vector<int>> tags;
        std::vector<int> currently_published_tags;
        std::vector<geometry_msgs::msg::Transform> tag_transforms;

        //tag types
        std::vector<std::string> tag_types;

        //tf
        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tag_pose_broadcaster;

        //tf functions
        bool is_transform_available(std::string from_frame, std::string to_frame, double max_time_ago);

        //update the tags in the removable list
        void update_removable_tags();


        private slots:
        //update combos
        void update_availble_tags(int index);

        //create tag
        void create_sim_april_tag();

        //remove tag
        void remove_tag();

        //remove all tags
        void remove_all_tags();

    };

} // namespace riptide_rviz
