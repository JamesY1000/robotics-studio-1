#ifndef OBJECT_LABEL_PLUGIN_HPP_
#define OBJECT_LABEL_PLUGIN_HPP_

#include <atomic>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/AxisAlignedBox.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/plugin/Register.hh>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sample_gen_msgs/msg/object_label.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace object_label_plugin
{

    class ObjectLabelPluginPrivate
    {
    public:
        ignition::gazebo::Entity link_;
        std::string topic_name;
        std::string tf_topic_name;
        std::string frame_name;
        std::string node_name;
        rclcpp::Publisher<sample_gen_msgs::msg::ObjectLabel>::SharedPtr object_label_pub;
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_model_pub;
        double update_rate;
        ignition::math::Pose3d pose;
        sample_gen_msgs::msg::ObjectLabel object_label_msg;
        geometry_msgs::msg::TransformStamped transform_stamped_msg;
        tf2_msgs::msg::TFMessage tf_msg;
        std::chrono::steady_clock::time_point last_time;
        bool bbox_calculated {false};
        geometry_msgs::msg::Vector3 bbox_size;
        const ignition::gazebo::components::Pose* poseComponent;
        ignition::math::AxisAlignedBox bBoxModel;
        /// @brief True once the ROS node + publishers exist (they are created
        /// asynchronously on a separate thread after Configure()).
        std::atomic<bool> ros_ready {false};
    };

    class ObjectLabelPlugin: public ignition::gazebo::System,
                             public ignition::gazebo::ISystemConfigure,
                             public ignition::gazebo::ISystemUpdate
    {
    public:
        ObjectLabelPlugin();
        ~ObjectLabelPlugin() override;

        void Configure(const ignition::gazebo::Entity& _entity,
                       const std::shared_ptr<const sdf::Element>& _sdf,
                       ignition::gazebo::EntityComponentManager& _ecm,
                       ignition::gazebo::EventManager& _eventMgr) override;

        void Update(const ignition::gazebo::UpdateInfo& _info,
                    ignition::gazebo::EntityComponentManager& _ecm) override;

    private:
        /// @brief Model interface
        ignition::gazebo::Model model {ignition::gazebo::kNullEntity};

        /// Private data pointer
        std::unique_ptr<ObjectLabelPluginPrivate> impl_;

        /// @brief ROS2 node
        rclcpp::Node::SharedPtr ros_node_;

        /// @brief ROS2 executor
        rclcpp::executors::MultiThreadedExecutor::SharedPtr ros_executor_;

        /// @brief Thread holding the ROS2 spinner
        std::thread spin_thread_;
    };
}   // namespace object_label_plugin

#endif  // OBJECT_LABEL_PLUGIN_HPP_
