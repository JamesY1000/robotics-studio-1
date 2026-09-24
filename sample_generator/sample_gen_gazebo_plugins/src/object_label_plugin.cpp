#include "object_label_plugin.hpp"

#include <mutex>

namespace object_label_plugin
{

    ObjectLabelPlugin::ObjectLabelPlugin()
        : impl_(std::make_unique<ObjectLabelPluginPrivate>())
    {
    }

    ObjectLabelPlugin::~ObjectLabelPlugin()
    {
        this->ros_executor_->remove_node(this->ros_node_);
        this->ros_executor_->cancel();
        this->spin_thread_.join();
    }

    void ObjectLabelPlugin::Configure(const ignition::gazebo::Entity& _entity,
                                      const std::shared_ptr<const sdf::Element>& _sdf,
                                      ignition::gazebo::EntityComponentManager& _ecm,
                                      ignition::gazebo::EventManager& /*_eventMgr*/)
    {
        std::string link_name;

        this->model = ignition::gazebo::Model(_entity);
        ignmsg << "Loading ObjectLabelPlugin for " << this->model.Name(_ecm).c_str() << std::endl;

        // Load parameters
        if (!_sdf->HasElement("linkName"))
        {
            link_name = this->model.Name(_ecm);
            ignmsg << "ObjectLabelPlugin missing <linkName>, please check specified link. Meanwhile, found model name "
                      "instead: "
                   << link_name.c_str() << std::endl;
        }
        else
        {
            link_name = _sdf->Get<std::string>("linkName");
        }

        ignmsg << "Attempting to find link by name: " << link_name << std::endl;
        impl_->link_ = this->model.LinkByName(_ecm, link_name);
        if (impl_->link_ == ignition::gazebo::kNullEntity)
        {
            ignerr << "ObjectLabelPlugin plugin error: linkName: " << link_name.c_str() << " does not exist"
                   << std::endl;
            return;
        }
        ignmsg << "Link found: " << link_name << std::endl;

        if (!_sdf->HasElement("topicName"))
        {
            ignmsg << "ObjectLabelPlugin missing <topicName>, defaults to " << this->model.Name(_ecm).c_str()
                   << "/object_label" << std::endl;
            impl_->topic_name = this->model.Name(_ecm) + "/object_label";
        }
        else
        {
            impl_->topic_name = _sdf->Get<std::string>("topicName");
        }

        if (!_sdf->HasElement("frameName"))
        {
            impl_->frame_name = this->model.Name(_ecm);
            ignmsg << "ObjectLabelPlugin missing <frameName>, defaults to " << impl_->frame_name.c_str() << std::endl;
        }
        else
        {
            impl_->frame_name = _sdf->Get<std::string>("frameName");
        }

        if (!_sdf->HasElement("updateRate"))
        {
            impl_->update_rate = 20.0;
            ignmsg << "ObjectLabelPlugin missing <updateRate>, defaults to " << impl_->update_rate << "Hz" << std::endl;
        }
        else
        {
            impl_->update_rate = _sdf->Get<std::double_t>("updateRate");
            ignmsg << "ObjectLabelPlugin update rate initialized with " << impl_->update_rate << std::endl;
        }

        if (!_sdf->HasElement("classification"))
        {
            ignmsg << "ObjectLabelPlugin missing <classification>, defaults to 0 (unclassified)" << std::endl;
            impl_->object_label_msg.classification = 0;
        }
        else
        {
            int classification_int = _sdf->Get<int>("classification");
            ignmsg << "ObjectLabelPlugin <classification> initialized to " << classification_int << std::endl;
            impl_->object_label_msg.classification = static_cast<uint8_t>(classification_int);
        }

        if (_sdf->HasElement("boxSizeX") || _sdf->HasElement("boxSizeY") || _sdf->HasElement("boxSizeZ"))
        {
            impl_->bbox_size.x = _sdf->Get<double>("boxSizeX");
            impl_->bbox_size.y = _sdf->Get<double>("boxSizeY");
            impl_->bbox_size.z = _sdf->Get<double>("boxSizeZ");
            ignmsg << "Using provided bounding box size X: " << impl_->bbox_size.x << " Y: " << impl_->bbox_size.y
                   << " Z: " << impl_->bbox_size.z << std::endl;
        }

        if (impl_->bbox_size.x < impl_->bbox_size.y)
        {
            ignwarn << "Width is larger than length. Object might be initialized wrong" << std::endl;
        }

        // Get information about bounding box and print out by calculation or from sdf (calculation unstable...)
        impl_->poseComponent = _ecm.Component<ignition::gazebo::components::Pose>(_entity);

        // Create component if it doesn't exist
        if (!impl_->poseComponent)
        {
            ignwarn << "ObjectLabelPlugin error: Pose component not found for entity " << _entity
                    << ". Creating component." << std::endl;
            _ecm.CreateComponent(_entity, ignition::gazebo::components::Pose());
        }

        impl_->pose = impl_->poseComponent->Data();
        ignmsg << "Gz sim pose: " << impl_->pose << std::endl;

        // Check if pitch and roll are non zero and print warning message if so
        if (impl_->pose.Rot().Pitch() != 0 || impl_->pose.Rot().Roll() != 0)
        {
            ignwarn << "Pitch or roll angle are not zero during initialization of object state plugin!" << std::endl;
            ignwarn << "Size of bounding box can not be trusted!" << std::endl;
        }

        // Retrieve the AxisAlignedBox component
        auto axisAlignedBoxComponent = _ecm.Component<ignition::gazebo::components::AxisAlignedBox>(_entity);

        // Initial check for AxisAlignedBox component - create component if it doesn't exist
        if (!axisAlignedBoxComponent)
        {
            ignwarn << "ObjectLabelPlugin error: AxisAlignedBox component not found for entity " << _entity
                    << ". Creating component." << std::endl;
            _ecm.CreateComponent(_entity, ignition::gazebo::components::AxisAlignedBox());
        }

        // Initialize last_time
        impl_->last_time = std::chrono::steady_clock::now();
        ignmsg << "Initialized last_time: "
               << std::chrono::duration_cast<std::chrono::seconds>(impl_->last_time.time_since_epoch()).count()
               << " seconds since epoch" << std::endl;

        // Start a separate thread to wait for ROS 2 initialization (otherwise, the plugin will not load properly)
        std::thread([this, &_ecm, &_entity]() {
            // impl_->node_name = "object_label_node_" + this->model.Name(_ecm);
            std::string model_name = this->model.Name(_ecm);
            std::replace(model_name.begin(), model_name.end(), '-', '_');
            impl_->node_name = "object_label_node_" + model_name;

            // The plugin runs inside the `ign gazebo` server process, which does
            // NOT initialize ROS 2 on its own. Without rclcpp::init() here,
            // rclcpp::ok() stays false forever, the node/publishers are never
            // created and Update() segfaults on a null publisher.
            // Several plugin instances (one per spawned tree) share this one
            // process, so the init must be serialised with a static guard -
            // concurrent rclcpp::init() from two model threads is a data race
            // and can corrupt the context (then publish() segfaults later).
            {
                static std::mutex g_ros_init_mutex;
                static bool g_ros_initialized = false;
                if (!g_ros_initialized)
                {
                    std::lock_guard<std::mutex> lock(g_ros_init_mutex);
                    if (!g_ros_initialized)
                    {
                        try
                        {
                            rclcpp::init(0, nullptr);
                        }
                        catch (const std::exception& e)
                        {
                            ignwarn << "[ObjectLabelPlugin] rclcpp::init failed "
                                    << "(already initialized?): " << e.what() << std::endl;
                        }
                        g_ros_initialized = true;
                    }
                }
            }

            while (!rclcpp::ok())
            {
                ignerr << "[ObjectLabelPlugin] Timeout waiting for ROS 2 initialization.";
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            ignmsg << "ROS 2 context available: " << impl_->node_name << std::endl;

            // Start ros2 stuff
            this->ros_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
            this->ros_node_     = rclcpp::Node::make_shared(impl_->node_name);
            this->ros_executor_->add_node(this->ros_node_);

            ignmsg << "Created ROS 2 node" << std::endl;

            // Spin executor in thread
            this->spin_thread_ = std::thread([this]() { this->ros_executor_->spin(); });

            ignmsg << "Spinning executor" << std::endl;

            // Initialise object label pub
            std::string topic_name = this->model.Name(_ecm);
            std::replace(topic_name.begin(), topic_name.end(), '-', '_');
            impl_->topic_name    = "/object_label/" + topic_name;
            impl_->tf_topic_name = "/tf_object_models";

            impl_->object_label_pub =
                ros_node_->create_publisher<sample_gen_msgs::msg::ObjectLabel>(impl_->topic_name, 10);

            impl_->tf_model_pub = ros_node_->create_publisher<tf2_msgs::msg::TFMessage>(impl_->tf_topic_name, 10);

            // Only now is it safe for Update() to publish.
            impl_->ros_ready.store(true);
        }).detach();

        ignmsg << "ObjectLabelPlugin for " << this->model.Name(_ecm).c_str() << " initialized" << std::endl;
    }

    void ObjectLabelPlugin::Update(const ignition::gazebo::UpdateInfo& _info,
                                   ignition::gazebo::EntityComponentManager& _ecm)
    {
        if (!impl_->link_)
        {
            return;
        }

        // Get updated pose at each simulation step
        impl_->pose = impl_->poseComponent->Data();

        // Convert time_point to duration since epoch and print
        auto currentTime = std::chrono::steady_clock::time_point(_info.simTime);

        if (currentTime < impl_->last_time)
        {
            impl_->last_time = currentTime;
            impl_->last_time = currentTime;
        }

        // Calculate the duration since the last update
        auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - impl_->last_time);

        // Calculate the expected interval based on the update rate
        double expectedInterval = 1.0 / impl_->update_rate;

        // Check if the elapsed time since the last update exceeds the expected interval
        if (impl_->update_rate > 0 && duration.count() < expectedInterval)
        {
            return;
        }

        if (!impl_->bbox_calculated)
        {
            auto axisAlignedBoxComponent =
                _ecm.Component<ignition::gazebo::components::AxisAlignedBox>(this->model.Entity());

            if (!axisAlignedBoxComponent)
            {
                ignerr << "AxisAlignedBoxComponent still doesn't exist after initial check and creation" << std::endl;
            }

            ignition::math::AxisAlignedBox bBoxModel = axisAlignedBoxComponent->Data();
            ignmsg << "BoundingBox Min: " << bBoxModel.Min() << " Max: " << bBoxModel.Max() << std::endl;

            // Set/create the objectlabel message

            // Calculate bounding box with new algorithm
            if (!impl_->bbox_size.x || !impl_->bbox_size.y || !impl_->bbox_size.z)
            {
                ignmsg << "ObjectLabelPlugin missing boundingBox information, initializing data from gz" << std::endl;

                double X = std::abs(bBoxModel.XLength());
                double Y = std::abs(bBoxModel.YLength());
                double sinYaw = sin(impl_->pose.Rot().Yaw());
                double cosYaw = cos(impl_->pose.Rot().Yaw());

                if (std::abs(X - Y) <= 1e-6 * std::max(X, Y))
                {
                    // Radially-symmetric object (e.g. a pine cone): its AABB does
                    // NOT change with yaw, so it already IS the intrinsic size.
                    // The "unrotate a box" formula below would under-estimate it
                    // (and give 0/0 at 45 deg), producing wrong/oversized boxes.
                    impl_->bbox_size.x = X;
                    impl_->bbox_size.y = Y;
                }
                else if (std::abs(cosYaw) < 1e-6)   // Close to -90° or 90°
                {
                    impl_->bbox_size.x = Y;   // Swap dimensions
                    impl_->bbox_size.y = X;
                }
                else
                {
                    double c2s2 = cosYaw * cosYaw - sinYaw * sinYaw;   // cos(2*yaw)
                    if (std::abs(c2s2) < 1e-6)
                    {
                        // Close to 45°/135°: the linear system is singular (0/0).
                        // Fall back to the AABB dims - a bounded over-estimate
                        // for an asymmetric box, but avoids NaN/garbage boxes.
                        impl_->bbox_size.x = X;
                        impl_->bbox_size.y = Y;
                    }
                    else
                    {
                        impl_->bbox_size.x = std::abs((cosYaw * X - sinYaw * Y) / c2s2);
                        impl_->bbox_size.y = std::abs((Y - sinYaw * impl_->bbox_size.x) / cosYaw);

                        // The unrotate formula above is only exact for box-shaped
                        // objects. For arbitrary meshes (trees) the world AABB does
                        // NOT follow the box formula, so near 45 deg the numerator
                        // stays finite while cos(2*yaw) -> 0 and the size explodes
                        // (e.g. a 10.6 m oak at 44 deg unrotates to ~49 m). A rotated
                        // box's AABB is always >= its intrinsic dims, so clamping to
                        // the AABB is a no-op for boxes and a bounded, sensible
                        // estimate for trees (whose AABB is already ~intrinsic size).
                        impl_->bbox_size.x = std::min(impl_->bbox_size.x, X);
                        impl_->bbox_size.y = std::min(impl_->bbox_size.y, Y);
                    }
                }

                impl_->bbox_size.z = std::abs(bBoxModel.ZLength());
                ignmsg << "Bounding box size X: " << impl_->bbox_size.x << " Y: " << impl_->bbox_size.y
                       << " Z: " << impl_->bbox_size.z << std::endl;
            }

            if (impl_->bbox_size.x < impl_->bbox_size.y)
            {
                ignwarn << "Width is larger than length. Object might be initialized wrong" << std::endl;
            }

            // Store values which do not change
            impl_->object_label_msg.size               = impl_->bbox_size;
            impl_->object_label_msg.header.frame_id    = impl_->frame_name;
            impl_->object_label_msg.pose.orientation.x = 0;
            impl_->object_label_msg.pose.orientation.y = 0;
            impl_->object_label_msg.pose.orientation.z = 0;
            impl_->object_label_msg.pose.orientation.w = 1;

            // Fill in pose (TF is going to be used to transform between frames)
            // Position (Use offset between collision and position)
            double sinYaw  = sin(impl_->pose.Rot().Yaw());
            double cosYaw  = cos(impl_->pose.Rot().Yaw());
            double offSetX = bBoxModel.Center().X() - impl_->pose.Pos().X();
            double offSetY = bBoxModel.Center().Y() - impl_->pose.Pos().Y();
            double offSetZ = bBoxModel.Center().Z() - impl_->pose.Pos().Z();

            ignmsg << "Offsets X: " << offSetX << " Y: " << offSetY << " Z: " << offSetZ << std::endl;

            impl_->object_label_msg.offset_x = offSetX;
            impl_->object_label_msg.offset_y = offSetY;
            impl_->object_label_msg.offset_z = offSetZ;

            impl_->object_label_msg.pose.position.x = sinYaw * offSetY + cosYaw * offSetX;
            impl_->object_label_msg.pose.position.y = cosYaw * offSetY - sinYaw * offSetX;
            impl_->object_label_msg.pose.position.z = bBoxModel.Center().Z() - impl_->pose.Pos().Z();

            impl_->bbox_calculated = true;
        }

        // Differentiate to get accelerations
        double tempDt = duration.count();
        if (tempDt != 0)
        {
            impl_->last_time = currentTime;
            auto sim_time    = std::chrono::steady_clock::time_point(_info.simTime);

            // Publish object label msg
            auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(sim_time.time_since_epoch()).count();
            auto now_nsec =
                std::chrono::duration_cast<std::chrono::nanoseconds>(sim_time.time_since_epoch()).count() % 1000000000;
            impl_->object_label_msg.header.stamp.sec     = now_sec;
            impl_->object_label_msg.header.stamp.nanosec = now_nsec;

            // The publishers are created asynchronously after Configure() - only
            // publish once they exist (avoids a null-pointer segfault that used
            // to kill the whole `ign gazebo` server on startup).
            if (impl_->ros_ready.load())
            {
                impl_->object_label_pub->publish(impl_->object_label_msg);
            }

            // Use tf to transform into 'world' frame
            ignition::math::Pose3d tf_pose = ignition::gazebo::worldPose(this->model.Entity(), _ecm);

            impl_->transform_stamped_msg.header.stamp.sec     = now_sec;
            impl_->transform_stamped_msg.header.stamp.nanosec = now_nsec;
            impl_->transform_stamped_msg.header.frame_id      = "world";
            impl_->transform_stamped_msg.child_frame_id       = impl_->frame_name;

            impl_->transform_stamped_msg.transform.translation.x = tf_pose.Pos().X();
            impl_->transform_stamped_msg.transform.translation.y = tf_pose.Pos().Y();
            impl_->transform_stamped_msg.transform.translation.z = tf_pose.Pos().Z();

            impl_->transform_stamped_msg.transform.rotation.x = tf_pose.Rot().X();
            impl_->transform_stamped_msg.transform.rotation.y = tf_pose.Rot().Y();
            impl_->transform_stamped_msg.transform.rotation.z = tf_pose.Rot().Z();
            impl_->transform_stamped_msg.transform.rotation.w = tf_pose.Rot().W();

            impl_->tf_msg.transforms = {impl_->transform_stamped_msg};

            if (impl_->ros_ready.load())
            {
                impl_->tf_model_pub->publish(impl_->tf_msg);
            }
        }
    }

    // Register plugin with the simulator (:: prefix required because the macro
    // expands inside a nested `ignition` namespace within `object_label_plugin`)
    IGNITION_ADD_PLUGIN(ObjectLabelPlugin,
                        ::ignition::gazebo::System,
                        ::ignition::gazebo::ISystemConfigure,
                        ::ignition::gazebo::ISystemUpdate)
}   // namespace object_label_plugin
