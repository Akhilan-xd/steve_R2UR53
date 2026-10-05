#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace steve_simulation
{

// Welds pick_cube to the gripper palm on request. The finger joints are
// position-locked, so a closing command cannot squeeze. This joint is what
// makes the cube travel with the arm. Cube collision is turned off while the
// weld exists, otherwise the fingers drive into the cube and the impulse
// throws the mobile base.
class GraspAttachPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    ros_node_ = gazebo_ros::Node::Get(sdf);
    parent_link_name_ = Element(sdf, "parent_link", "ur5ewrist_3_link");
    child_model_name_ = Element(sdf, "child_model", "pick_cube");
    child_link_name_ = Element(sdf, "child_link", "cube");

    service_ = ros_node_->create_service<std_srvs::srv::SetBool>(
      "/grasp_cube",
      std::bind(&GraspAttachPlugin::OnService, this, std::placeholders::_1, std::placeholders::_2));
    // Cube center in the palm frame. The pick script compares it with
    // gripper_tcp to decide whether the fingers are really around the cube.
    in_palm_ = ros_node_->create_publisher<geometry_msgs::msg::PointStamped>(
      "/grasp_cube/cube_in_palm", 10);

    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GraspAttachPlugin::OnUpdate, this));

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Grasp weld ready. Parent %s, child %s::%s",
      parent_link_name_.c_str(), child_model_name_.c_str(), child_link_name_.c_str());
  }

private:
  static std::string Element(sdf::ElementPtr sdf, const char * key, const char * fallback)
  {
    if (sdf && sdf->HasElement(key)) {
      return sdf->Get<std::string>(key);
    }
    return fallback;
  }

  void OnService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_hold_ = request->data;
      pending_ = true;
      finished_ = false;
      ok_ = false;
      message_.clear();
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (finished_) {
          response->success = ok_;
          response->message = message_;
          return;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    response->success = false;
    response->message = "grasp weld timed out";
  }

  void PublishInPalm()
  {
    auto world = model_->GetWorld();
    if (!world || !in_palm_) {
      return;
    }
    const auto now = world->SimTime();
    if ((now - last_publish_).Double() < 0.05) {
      return;
    }
    last_publish_ = now;
    auto palm = FindLink(model_, parent_link_name_);
    auto cube = CubeLink();
    if (!palm || !cube) {
      return;
    }
    const auto offset = cube->WorldPose().Pos() - palm->WorldPose().Pos();
    const auto local = palm->WorldPose().Rot().RotateVectorReverse(offset);
    geometry_msgs::msg::PointStamped msg;
    msg.header.frame_id = parent_link_name_;
    msg.header.stamp.sec = static_cast<int32_t>(now.sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>(now.nsec);
    msg.point.x = local.X();
    msg.point.y = local.Y();
    msg.point.z = local.Z();
    in_palm_->publish(msg);
  }

  void OnUpdate()
  {
    PublishInPalm();
    bool have_request = false;
    bool hold = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!pending_) {
        return;
      }
      pending_ = false;
      have_request = true;
      hold = pending_hold_;
    }
    if (!have_request) {
      return;
    }

    std::string message;
    const bool ok = hold ? Attach(message) : Detach(message);

    std::lock_guard<std::mutex> lock(mutex_);
    ok_ = ok;
    message_ = message;
    finished_ = true;
  }

  // Gazebo's link map is keyed by scoped names (mmo_700::robotiq_85_base_link).
  // GetLink("robotiq_85_base_link") misses that entry.
  static gazebo::physics::LinkPtr FindLink(
    const gazebo::physics::ModelPtr & model, const std::string & name)
  {
    if (!model || name.empty()) {
      return nullptr;
    }
    if (auto link = model->GetLink(name)) {
      return link;
    }
    const std::string scoped = model->GetName() + "::" + name;
    if (auto link = model->GetLink(scoped)) {
      return link;
    }
    for (const auto & link : model->GetLinks()) {
      if (!link) {
        continue;
      }
      const std::string link_name = link->GetName();
      const std::string scoped_name = link->GetScopedName();
      if (link_name == name || scoped_name == name || scoped_name == scoped ||
        (link_name.size() >= name.size() &&
        link_name.compare(link_name.size() - name.size(), name.size(), name) == 0))
      {
        return link;
      }
    }
    for (const auto & nested : model->NestedModels()) {
      if (auto link = FindLink(nested, name)) {
        return link;
      }
    }
    return nullptr;
  }

  gazebo::physics::ModelPtr FindModel(const std::string & name) const
  {
    auto world = model_->GetWorld();
    if (!world || name.empty()) {
      return nullptr;
    }
    if (auto found = world->ModelByName(name)) {
      return found;
    }
    for (const auto & candidate : world->Models()) {
      if (!candidate) {
        continue;
      }
      if (candidate->GetName() == name || candidate->GetScopedName() == name) {
        return candidate;
      }
      for (const auto & nested : candidate->NestedModels()) {
        if (nested && (nested->GetName() == name || nested->GetScopedName() == name)) {
          return nested;
        }
      }
    }
    return nullptr;
  }

  gazebo::physics::LinkPtr CubeLink() const
  {
    return FindLink(FindModel(child_model_name_), child_link_name_);
  }

  bool Attach(std::string & message)
  {
    if (joint_) {
      message = "cube already welded to the gripper";
      return true;
    }

    auto parent = FindLink(model_, parent_link_name_);
    auto child_model = FindModel(child_model_name_);
    auto child = FindLink(child_model, child_link_name_);
    if (!parent || !child) {
      message = "missing ";
      message += parent ? "" : ("palm link " + parent_link_name_ + " on " + model_->GetName());
      message += (!parent && !child) ? " and " : "";
      message += child ? "" : ("cube " + child_model_name_ + "::" + child_link_name_);
      RCLCPP_ERROR(ros_node_->get_logger(), "%s", message.c_str());
      return false;
    }

    const ignition::math::Pose3d held_pose = child->WorldPose();
    auto physics = model_->GetWorld()->Physics();
    joint_ = physics->CreateJoint("fixed", model_);
    joint_->SetName("steve_grasp_cube");
    joint_->Attach(parent, child);
    joint_->Load(parent, child, ignition::math::Pose3d::Zero);
    joint_->Init();

    child->SetGravityMode(false);
    child->SetCollideMode("none");
    child->SetLinearVel(ignition::math::Vector3d::Zero);
    child->SetAngularVel(ignition::math::Vector3d::Zero);
    child->SetWorldPose(held_pose);

    message = "welded pick_cube to the gripper";
    RCLCPP_INFO(ros_node_->get_logger(), "%s", message.c_str());
    return true;
  }

  bool Detach(std::string & message)
  {
    auto child = CubeLink();
    if (child) {
      child->SetCollideMode("all");
      child->SetGravityMode(true);
    }

    if (joint_) {
      joint_->Detach();
      model_->RemoveJoint(joint_->GetName());
      joint_.reset();
      message = "released pick_cube";
      RCLCPP_INFO(ros_node_->get_logger(), "%s", message.c_str());
    } else {
      message = "pick_cube was not welded";
    }
    return true;
  }

  gazebo::physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr in_palm_;
  gazebo::common::Time last_publish_;
  gazebo::event::ConnectionPtr update_;
  gazebo::physics::JointPtr joint_;

  std::string parent_link_name_;
  std::string child_model_name_;
  std::string child_link_name_;

  std::mutex mutex_;
  bool pending_{false};
  bool pending_hold_{false};
  bool finished_{false};
  bool ok_{false};
  std::string message_;
};

}  // namespace steve_simulation

GZ_REGISTER_MODEL_PLUGIN(steve_simulation::GraspAttachPlugin)
