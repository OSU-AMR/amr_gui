#include "amrviz/Apriltag.hpp"
#include <chrono>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;


namespace amrviz
{
    Apriltag::Apriltag(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        uiPanel = new Ui_Apriltag();
        uiPanel->setupUi(this);

        RVIZ_COMMON_LOG_INFO("Apriltag Panel: Constructed apriltag panel");

    }


    Apriltag::~Apriltag()
    {
        // master window control removal
        delete uiPanel;
    }


    void Apriltag::onInitialize()
    {
        //connect buttons to things

        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        node->declare_parameter(TAG_STALE_PARAM_NAME, 5);

        //load in april tags
        load_in_tags();

        //listen for the transforms
        tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        //init broadcaster
        tag_pose_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*node);

        //robot list refresh timer
        robot_list_timer = node->create_wall_timer(1000ms, std::bind(&Apriltag::refresh_robot_list, this));
        robot_sensor_update_timer = node->create_wall_timer(1000ms, std::bind(&Apriltag::refresh_robot_list, this));
        update_published_tags_timer = node->create_wall_timer(1000ms, std::bind(&Apriltag::repub_tags, this));

        //set the tag type combo box
        setComboBoxItems(uiPanel->tagtypecombo, tag_types);
        connect(uiPanel->tagtypecombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &Apriltag::update_availble_tags);

        //set the function to the create tag button
        connect(uiPanel->placebutton, &QPushButton::clicked, this, &Apriltag::create_sim_april_tag);

        //connect remove buttons
        connect(uiPanel->removetagbutton, &QPushButton::clicked, this, &Apriltag::remove_tag);
        connect(uiPanel->removeallbutton, &QPushButton::clicked, this, &Apriltag::remove_all_tags);

        RVIZ_COMMON_LOG_INFO("Apriltag Panel: Inited apriltag panel");

        try{
            //may error if declared by another panel and thats ok
            node->declare_parameter("focus_robot", "");
        } catch(...){}

    }


    void Apriltag::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);
    }

    void Apriltag::update_robot_based_subscribers(){
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        std::string selected_robot = get_current_robot();

        if(uiPanel->focus_box->isChecked()){
            node->get_parameter("focus_robot", selected_robot);
        }


        if(selected_robot != ""){
            //update the ros2 publishers
            std::string slash_str = "/";
            //EXAMPLE SUB RFID_sub = node->create_subscription<std_msgs::msg::String>(slash_str + selected_robot + RFID_TOPIC, rclcpp::SystemDefaultsQoS(), std::bind(&Sensors::RFID_update_cb, this, _1)); 
        }    

    }

    void Apriltag::refresh_robot_list(){
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
    std::vector<std::string> Apriltag::split_topics_into_componets(const std::string& topic) {
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

    void Apriltag::removeDuplicateStrings(std::vector<std::string>& vec) {
        std::unordered_set<std::string> seen;
        std::vector<std::string> result;

        for (const auto& str : vec) {
            if (seen.insert(str).second) {  // insert returns pair<iterator, bool>
                result.push_back(str);
            }
        }

        vec = std::move(result);
    }

    void Apriltag::setComboBoxItems(QComboBox* comboBox, const std::vector<std::string>& items) {
        if (!comboBox) return;  // Safety check

        comboBox->clear();  // Remove existing items

        //add in the default item
        comboBox->addItem("");

        for (const auto& item : items) {
            comboBox->addItem(QString::fromStdString(item));
        }
    }

    std::string Apriltag::get_current_robot(){
        //returns the current selected robot
        //NOTE: the selected robot can be "" which indicates no robot selected

        return uiPanel->robot_select_block->currentText().toStdString();
    }

    std::string Apriltag::get_selected_tag_type(){      
        //get the current tag type string
        return uiPanel->tagtypecombo->currentText().toStdString();
    }

    double Apriltag::get_tag_x_position(){
        //get the tag x position
        //not a number? return zero and wine
        try{
            return std::stod(uiPanel->tagpositionxbox->text().toStdString());
        }catch(...){
            RVIZ_COMMON_LOG_INFO("Didn't reviece a number for an x position, setting to 0");
            return 0;
        }
    }

    double Apriltag::get_tag_y_position(){
        //get the tag x position
        //not a number? return zero and wine
        try{
            return std::stod(uiPanel->tagpositionybox->text().toStdString());
        }catch(...){
            RVIZ_COMMON_LOG_INFO("Didn't reviece a number for an y position, setting to 0");
            return 0;
        }
    }

    std::string Apriltag::get_selected_tag(){
        //get the selected tag type string

        return uiPanel->tagidcombo->currentText().toStdString();
    }

    std::string Apriltag::get_remove_tag(){
        //get the selected tag to be removed string

        return uiPanel->tagidtoremove->currentText().toStdString();
    }


    void Apriltag::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);

    }

    double Apriltag::get_ros2_time()
    {
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        rclcpp::Time now = node->get_clock()->now();

        // Combine seconds and nanoseconds into a single float (fractional seconds)
        double ns = now.nanoseconds();

        return ns / 1000000000.0;
    }

    void Apriltag::create_sim_april_tag(){
        //get the tag from combo
        std::string selected_tag = get_selected_tag();
        int tag_id = get_id_from_tag_string(selected_tag);

        if(tag_id <= 0){
            RVIZ_COMMON_LOG_WARNING("Invalid tag id passed, not creating!");

            return;
        }

        // filter to ensure this tag is available
        std::vector<int> tag_array;
        tag_array.push_back(tag_id);
        if(!(filter_available_tags(tag_array).size() > 0)){
            //tag is unavailable
            RVIZ_COMMON_LOG_WARNING("Tag has become unavailable, not creating!");
            return;
        }

        //fill out a transform to put the tag at
        geometry_msgs::msg::Transform transform;
        transform.translation.x = get_tag_x_position();
        transform.translation.y = get_tag_y_position();
        transform.translation.z = 0;
        transform.rotation.x = 0;
        transform.rotation.y = 0;
        transform.rotation.z = 0;
        transform.rotation.w = 1;

        //add to the list
        currently_published_tags.push_back(tag_id);
        tag_transforms.push_back(transform);

        //refresh the available list
        update_availble_tags(0);

        //update the list of tags to be removed
        update_removable_tags();
    }

    void Apriltag::update_removable_tags(){

        //make list of strings
        std::vector<std::string> removable_tags_list;
        for(int i = 0; i < currently_published_tags.size(); i++){
            removable_tags_list.push_back("tag_" + std::to_string(currently_published_tags.at(i)));
        }

        //update the combo
        setComboBoxItems(uiPanel->tagidtoremove, removable_tags_list);
    }

    void Apriltag::repub_tags(){
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        //republish the april tag transforms
        for(int i = 0; i < currently_published_tags.size(); i++){
            //fillout transform header
            geometry_msgs::msg::TransformStamped t;
            t.transform = tag_transforms.at(i);
            t.child_frame_id = "tag_" + std::to_string(currently_published_tags.at(i)) + "_sim";
            t.header.frame_id = MAP_FRAME;
            t.header.stamp = node->get_clock()->now();

            //send the transform
            tag_pose_broadcaster->sendTransform(t);
        }
    }


    void Apriltag::update_availble_tags(int index){
        //get value in the combo box
        std::string selected_type = get_selected_tag_type();

        //if tag type is empty
        if(selected_type == ""){
            return ;
        }

        //get index of list
        int tag_list_index = -1;
        for(int i = 0; i < tag_types.size(); i++){
            if(tag_types.at(i) == selected_type){
                tag_list_index = i;
                break;
            }
        }
        
        if(tag_list_index < 0){
            RVIZ_COMMON_LOG_WARNING("Odd, non registed tag type selected...");
            return;
        }

        //get the available tag numbers
        std::vector<int> tags_to_display = filter_available_tags(tags.at(tag_list_index));

        //get there display strings
        std::vector<std::string> tag_strings;
        for(int i = 0; i < tags_to_display.size(); i++){
            tag_strings.push_back("tag_" + std::to_string(tags_to_display.at(i)));
        }

        //update the combo box
        setComboBoxItems(uiPanel->tagidcombo, tag_strings);
    }

    void Apriltag::remove_all_tags(){
        //remove all tags from the refresh list
        currently_published_tags = std::vector<int>();
        tag_transforms = std::vector<geometry_msgs::msg::Transform>();

        //refresh the available list
        update_availble_tags(0);

        //update the list of tags to be removed
        update_removable_tags();
    }

    void Apriltag::remove_tag(){
        //remote a particular tag from the refresh list
        std::string tag_string =  get_remove_tag();

        if(tag_string == ""){
            RVIZ_COMMON_LOG_WARNING("Not removing tag as id is enmpty string");
            return;
        }

        //get the tag's id
        int tag_id = get_id_from_tag_string(tag_string);

        if(tag_id <= 0){
            RVIZ_COMMON_LOG_WARNING("Not removing tag as id is invalid");
            return;
        }

        //remove the id from the list
        int remove_index = -1;
        for(int i = 0; i < tag_id; i++){
            if(tag_id == currently_published_tags.at(i)){
                
                currently_published_tags.erase(currently_published_tags.begin() + i);
                tag_transforms.erase(tag_transforms.begin() + i);

                break;
            }
        }

        //refresh the available list
        update_availble_tags(0);

        //update the list of tags to be removed
        update_removable_tags();
    }

    bool Apriltag::is_transform_available(std::string from_frame, std::string to_frame, double max_time_ago){
        //try to perform the transform, if it valid, return true
        try{
            geometry_msgs::msg::TransformStamped t;
            t = tf_buffer->lookupTransform(to_frame, from_frame,tf2::TimePointZero);
            
            //transform is recent
            if(t.header.stamp.sec + t.header.stamp.nanosec / 1000000000.0 + max_time_ago > get_ros2_time()){
                
                //likely being currently published
                return true;
            }

            return false;
        } catch (const tf2::TransformException & ex) {
            //no transform, not available
            return false;
        }

        return false;
    }

    std::vector<int> Apriltag::filter_available_tags(std::vector<int> tags_to_filter){
        //filter for tags that are currently not being published

        std::vector<int> available_tags;

        for (int i = 0; i < tags_to_filter.size(); i++){

            std::string tag_frame = "tag_" + std::to_string(tags_to_filter.at(i));

            //no other node is publishing this tag
            if(!is_transform_available(MAP_FRAME, tag_frame, getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node()->get_parameter(TAG_STALE_PARAM_NAME).as_int())){

                //check to ensure that this tag is not being published by this node
                bool not_currently_published = true;
                for(int ii = 0; ii < currently_published_tags.size(); ii++){

                    if(currently_published_tags.at(ii) == tags_to_filter.at(i)){
                        not_currently_published = false;
                        break;
                    }
                }

                //add to available tags 
                if(not_currently_published){
                    available_tags.push_back(tags_to_filter.at(i));
                }
            }

        }

        return available_tags;
    }


    void Apriltag::load_in_tags()
    {
        //load in the yaml containing the tag data
        std::string share_dir_path = ament_index_cpp::get_package_share_directory("amr_central");
        YAML::Node tag_config;
        try{
            tag_config = YAML::LoadFile(share_dir_path + TAG_LIST_SUBPATH);
        }catch(...){
            RVIZ_COMMON_LOG_WARNING("Failed to load path at: " + share_dir_path + TAG_LIST_SUBPATH);
            return;
        }

        //load in the robot tags
        try{
            std::vector<int> robot_tags= tag_config["robot_tags"].as<std::vector<int>>();
            tags.push_back(robot_tags);

            tag_types.push_back("robot");
        }catch(...){
            RVIZ_COMMON_LOG_ERROR("Failed to parse robot tags!");
        }

        //load in obstacle tags
        try{
            std::vector<int> obstacle_tags = tag_config["obstacle_tags"].as<std::vector<int>>();

            //loop through tags
            for(int i = 0; i < obstacle_tags.size(); i++){

                std::string tag_name = tag_config["tag_" + std::to_string(obstacle_tags.at(i))]["name"].as<std::string>();

                std::string class_name;
                if(getClassTagIndex(tag_name, tag_types, &class_name) < 0){
                    //make a new class
                    std::vector<int> new_class;
                    new_class.push_back(obstacle_tags.at(i));
                    tags.push_back(new_class);

                    //add class type
                    tag_types.push_back(class_name);
                }else{
                    tags.at(getClassTagIndex(tag_name, tag_types, &class_name)).push_back(obstacle_tags.at(i));
                }
            }

        }catch(...){
            RVIZ_COMMON_LOG_ERROR("Failed to parse robot tags!");
        }

    }

    int Apriltag::getClassTagIndex(const std::string& new_tag, const std::vector<std::string>& classes, std::string* class_name) {
        //split the new tag name by "_"
        std::vector<std::string> result;
        std::stringstream ss(new_tag);
        std::string token;

        while (std::getline(ss, token, '_')) {
            result.push_back(token);
        }

        //test to see if the string matches on in the already defined classes
        for(int i = 0; i < classes.size(); i++){
            if(result.at(0) + "_" + result.at(1) == classes.at(i)){
                //if the tag class matches one thats already added, return that index
                return i;
            }
        }

        //determine the class name
        *class_name = result.at(0) + "_" + result.at(1);

        //return -1 to make a new class
        return -1;
    }

    int Apriltag::get_id_from_tag_string(const std::string& tag_string) {
        //split the new tag name by "_"
        std::vector<std::string> result;
        std::stringstream ss(tag_string);
        std::string token;

        while (std::getline(ss, token, '_')) {
            result.push_back(token);
        }

        try{
            return std::stoi(result.at(1));
        }catch(...){
            //not a int passed
            return -1;
        }
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(amrviz::Apriltag, rviz_common::Panel);
