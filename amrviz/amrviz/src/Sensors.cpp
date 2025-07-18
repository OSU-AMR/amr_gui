#include "amrviz/Sensors.hpp"
#include <chrono>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;


namespace amrviz
{
    Sensors::Sensors(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        uiPanel = new Ui_Sensors();
        uiPanel->setupUi(this);

        RVIZ_COMMON_LOG_INFO("Sensor Panel: Constructed sensor panel");

    }


    Sensors::~Sensors()
    {
        // master window control removal
        delete uiPanel;
    }


    void Sensors::onInitialize()
    {
        //connect buttons to things

        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        //robot list refresh timer
        robot_list_timer = node->create_wall_timer(1000ms, std::bind(&Sensors::refresh_robot_list, this));
        robot_sensor_update_timer = node->create_wall_timer(1000ms, std::bind(&Sensors::refresh_robot_list, this));

        RVIZ_COMMON_LOG_INFO("Sensor Panel: Inited sensor panel");

        //declare the focused robot parameter
        try{
            //may error if declared by another panel and thats ok
            node->declare_parameter("focus_robot", "");
        } catch(...){}
    }


    void Sensors::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);
    }

    void Sensors::update_robot_based_subscribers(){
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        std::string selected_robot = get_current_robot();

        if(uiPanel->focus_box->isChecked()){
            node->get_parameter("focus_robot", selected_robot);
        }

        if(selected_robot != "" && current_robot != selected_robot){
            //update the ros2 publishers
            std::string slash_str = "/";
            

            RFID_sub = node->create_subscription<std_msgs::msg::String>(slash_str + selected_robot + RFID_TOPIC, rclcpp::SystemDefaultsQoS(), std::bind(&Sensors::RFID_update_cb, this, _1)); 
            IR_sub_L = node->create_subscription<std_msgs::msg::UInt16>(slash_str + selected_robot + LEFT_IR_TOPIC, rclcpp::SystemDefaultsQoS(), std::bind(&Sensors::IR_left_update_cb, this, _1)); 
            IR_sub_R = node->create_subscription<std_msgs::msg::UInt16>(slash_str + selected_robot + RIGHT_IR_TOPIC, rclcpp::SystemDefaultsQoS(), std::bind(&Sensors::IR_right_update_cb, this, _1)); 
            encoder_sub = node->create_subscription<amr_msgs::msg::Encoder>(slash_str + selected_robot + ENCODER_TOPIC, rclcpp::SystemDefaultsQoS(), std::bind(&Sensors::encoder_update_cb, this, _1)); 
        }    

        current_robot = selected_robot;

    }

    void Sensors::RFID_update_cb(const std_msgs::msg::String& msg){
        //update the RFID data out
        uiPanel->rfid_tag_block->setText(QString::fromStdString(msg.data));

        uiPanel->rfid_time_block->setText(QString::fromStdString(std::to_string(get_ros2_time())));
    }

    void Sensors::IR_left_update_cb(const std_msgs::msg::UInt16& msg){
        //update the RFID data out
        uiPanel->IR_left_block->setText(QString::fromStdString(std::to_string(msg.data)));
    }

    void Sensors::IR_right_update_cb(const std_msgs::msg::UInt16& msg){
        //update the RFID data out
        uiPanel->IR_right_block->setText(QString::fromStdString(std::to_string(msg.data)));
    }

    void Sensors::encoder_update_cb(const amr_msgs::msg::Encoder& msg){
        //update the RFID data out
        uiPanel->encoder_left_block->setText(QString::fromStdString(std::to_string(msg.left_angle)));
        uiPanel->encoder_right_block->setText(QString::fromStdString(std::to_string(msg.right_angle)));
    }

    void Sensors::refresh_robot_list(){
        //refresh the list of topics
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        std::vector<std::string> robot_namespaces;

        //get the previous item
        std::string previous_selected_robot = get_current_robot();

        //detect if user focus robot
        if(uiPanel->focus_box->isChecked()){
            std::string focus_robot;
            node->get_parameter("focus_robot", focus_robot);

            if(get_current_robot() != focus_robot){
                node->set_parameter(rclcpp::Parameter("focus_robot", get_current_robot()));
            }
        }

        //sort by namespace
        auto topic_names_and_types = node->get_topic_names_and_types();

        for (const auto& [topic_name, topic_type] : topic_names_and_types) {
           std::string topic = topic_name;

           if (topic.find(HEARTBEAT_STRING) != std::string::npos) {
                std::vector<std::string> components = split_topics_into_componets(topic);
                robot_namespaces.push_back(components.at(0));
           }
        }

        //add in previous selected robot to allow data to be viewed after robot is disconnected
        robot_namespaces.push_back(previous_selected_robot);
        
        //remove any duplicates
        removeDuplicateStrings(robot_namespaces);

        //set the combo box items
        setComboBoxItems(uiPanel->robot_select_block, robot_namespaces);   

        //ensure the previous robot remains selected
        int index = uiPanel->robot_select_block->findText(QString::fromStdString(previous_selected_robot));
        if ( index != -1 ) { // -1 for not found
            uiPanel->robot_select_block->setCurrentIndex(index);
        }

        update_robot_based_subscribers();
    }

    //whos a good llm
    std::vector<std::string> Sensors::split_topics_into_componets(const std::string& topic) {
        std::vector<std::string> tokens;
        std::string current_token;
        for (char c : topic) {
            if (c == '/') {
                if (!current_token.empty()) {
                    tokens.push_back(current_token);
                    current_token.clear();
                }
            } else {
                current_token += c;
            }
        }
        if (!current_token.empty()) {
            tokens.push_back(current_token);
        }
        return tokens;
    }

    void Sensors::removeDuplicateStrings(std::vector<std::string>& vec) {
        std::unordered_set<std::string> seen;
        std::vector<std::string> result;

        for (const auto& str : vec) {
            if (seen.insert(str).second) {  // insert returns pair<iterator, bool>
                result.push_back(str);
            }
        }

        vec = std::move(result);
    }

    void Sensors::setComboBoxItems(QComboBox* comboBox, const std::vector<std::string>& items) {
        if (!comboBox) return;  // Safety check

        comboBox->clear();  // Remove existing items

        //add in the default item
        comboBox->addItem("");

        for (const auto& item : items) {
            comboBox->addItem(QString::fromStdString(item));
        }
    }

    std::string Sensors::get_current_robot(){
        //returns the current selected robot
        //NOTE: the selected robot can be "" which indicates no robot selected

        return uiPanel->robot_select_block->currentText().toStdString();
    }


    void Sensors::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);

    }

    double Sensors::get_ros2_time()
    {
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        rclcpp::Time now = node->get_clock()->now();

        // Combine seconds and nanoseconds into a single float (fractional seconds)
        double ns = now.nanoseconds();

        return ns / 1000000000.0;
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(amrviz::Sensors, rviz_common::Panel);
