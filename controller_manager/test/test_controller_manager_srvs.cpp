// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "controller_manager_test_common.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_manager/controller_loader_interface.hpp"
#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/list_controller_types.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using ::testing::_;
using ::testing::Return;

using namespace std::chrono_literals;


class TestControllerManagerSrvs : public TestControllerManager
{
public:
  TestControllerManagerSrvs()
  {
  }

  void SetUp() override
  {
    TestControllerManager::SetUp();
    cm_ = std::make_shared<controller_manager::ControllerManager>(
      robot_, executor_,
      "test_controller_manager");
    update_timer_ = cm_->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&controller_manager::ControllerManager::update, cm_.get()));

    executor_->add_node(cm_);

    EXPECT_EQ(controller_interface::return_type::SUCCESS, cm_->configure());
    executor_spin_future_ = std::async(
      std::launch::async, [this]() -> void {
        executor_->spin();
      });
    // This sleep is needed to prevent a too fast test from ending before the
    // executor has began to spin, which causes it to hang
    std::this_thread::sleep_for(50ms);
  }

  void TearDown() override
  {
    executor_->cancel();
  }

  template<typename T>
  std::shared_ptr<typename T::Response> call_service_and_wait(
    rclcpp::Client<T> & client,
    std::shared_ptr<typename T::Request> request,
    rclcpp::Executor & service_executor,
    bool update_controller_while_spinning = false)
  {
    EXPECT_TRUE(client.wait_for_service(std::chrono::milliseconds(500)));
    auto result = client.async_send_request(request);
    // Wait for the result.
    if (update_controller_while_spinning) {
      while (service_executor.spin_until_future_complete(
          result,
          50ms) != rclcpp::FutureReturnCode::SUCCESS)
      {
        cm_->update();
      }
    } else {
      EXPECT_EQ(
        service_executor.spin_until_future_complete(result),
        rclcpp::FutureReturnCode::SUCCESS);
    }
    return result.get();
  }

protected:
  std::shared_ptr<controller_manager::ControllerManager> cm_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  std::future<void> executor_spin_future_;
};

TEST_F(TestControllerManagerSrvs, list_controller_types)
{
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::ListControllerTypes>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::ListControllerTypes>(
    "list_controller_types");
  auto request = std::make_shared<controller_manager_msgs::srv::ListControllerTypes::Request>();

  auto result = call_service_and_wait(*client, request, srv_executor);
  // Number depends on the controllers that exist on the system
  size_t controller_types = result->types.size();
  ASSERT_GE(
    controller_types,
    1u);
  ASSERT_EQ(
    controller_types,
    result->base_classes.size());
  ASSERT_THAT(result->types, ::testing::Contains("test_controller"));
  ASSERT_THAT(
    result->base_classes,
    ::testing::Contains("controller_interface::ControllerInterface"));

  std::shared_ptr<ControllerLoaderMock> mock_loader(new ControllerLoaderMock);

  cm_->register_controller_loader(mock_loader);
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(
    controller_types + 1,
    result->types.size());
  ASSERT_EQ(
    result->types.size(),
    result->base_classes.size());
  ASSERT_THAT(result->types, ::testing::Contains("mock_test_controller"));
  ASSERT_THAT(
    result->base_classes,
    ::testing::Contains("controller_interface::MockControllerInterface"));
}


TEST_F(TestControllerManagerSrvs, list_controllers_srv) {
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::ListControllers>("list_controllers");
  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();

  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(
    0u,
    result->controller.size());

  auto test_controller = std::make_shared<test_controller::TestController>();
  auto abstract_test_controller = cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_TYPE);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(
    1u,
    result->controller.size());
  ASSERT_EQ(test_controller::TEST_CONTROLLER_NAME, result->controller[0].name);
  ASSERT_EQ(test_controller::TEST_CONTROLLER_TYPE, result->controller[0].type);
  ASSERT_EQ("inactive", result->controller[0].state);

  cm_->switch_controller(
    {test_controller::TEST_CONTROLLER_NAME}, {},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true,
    rclcpp::Duration(0, 0));

  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(
    1u,
    result->controller.size());
  ASSERT_EQ("active", result->controller[0].state);


  cm_->switch_controller(
    {}, {test_controller::TEST_CONTROLLER_NAME},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true,
    rclcpp::Duration(0, 0));

  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(
    1u,
    result->controller.size());
  ASSERT_EQ("inactive", result->controller[0].state);

  ASSERT_EQ(
    controller_interface::return_type::SUCCESS,
    cm_->unload_controller(test_controller::TEST_CONTROLLER_NAME));
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(
    0u,
    result->controller.size());
}

TEST_F(TestControllerManagerSrvs, reload_controller_libraries_srv) {
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::ReloadControllerLibraries>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::ReloadControllerLibraries>(
    "reload_controller_libraries");
  auto request =
    std::make_shared<controller_manager_msgs::srv::ReloadControllerLibraries::Request>();


  std::shared_ptr<ControllerLoaderMock> mock_loader(new ControllerLoaderMock);

  cm_->register_controller_loader(mock_loader);

  // Reload with no controllers running
  request->force_kill = false;
  EXPECT_CALL(*mock_loader, is_available(_)).WillRepeatedly(Return(false));
  EXPECT_CALL(*mock_loader, reload).Times(1);
  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_TRUE(result->ok);


  // Add a controller, but stopped
  auto test_controller = cm_->load_controller(
    test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_TYPE);

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    test_controller->get_lifecycle_node()->get_current_state().id());
  ASSERT_GT(
    test_controller.use_count(),
    1) << "Controller manager should have have a copy of this shared ptr";

  request->force_kill = false;
  EXPECT_CALL(*mock_loader, reload).Times(1);
  RCLCPP_INFO(cm_->get_logger(), "Doing reload");
  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    test_controller->get_lifecycle_node()->get_current_state().id());
  ASSERT_EQ(
    test_controller.use_count(),
    1) << "No more references to the controller after reloading.";
  test_controller.reset();

  test_controller = cm_->load_controller(
    test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_TYPE);
  // Start Controller
  cm_->switch_controller(
    {test_controller::TEST_CONTROLLER_NAME}, {},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true,
    rclcpp::Duration(0, 0));
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_controller->get_lifecycle_node()->get_current_state().id());


  // Failed reload due to active controller
  request->force_kill = false;
  EXPECT_CALL(*mock_loader, reload).Times(0);
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_FALSE(result->ok) << "Cannot reload if controllers are running";
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_controller->get_lifecycle_node()->get_current_state().id());
  ASSERT_GT(
    test_controller.use_count(),
    1) <<
    "Controller manager should still have have a copy of "
    "this shared ptr, no unloading was performed";

  // Force stop active controller
  request->force_kill = true;
  EXPECT_CALL(*mock_loader, reload).Times(1);
  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);


  ASSERT_EQ(
    test_controller.use_count(),
    1) << "No more references to the controller after reloading.";
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    test_controller->get_lifecycle_node()->get_current_state().id()) <<
    "Controller should have been stopped and cleaned up with force_kill = true";
}

TEST_F(TestControllerManagerSrvs, load_controller_srv) {
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::LoadController>(
    "load_controller");

  auto request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
  request->name = test_controller::TEST_CONTROLLER_NAME;
  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_FALSE(result->ok) << "There's no param specifying the type for " << request->name;
  rclcpp::Parameter joint_parameters(std::string(test_controller::TEST_CONTROLLER_NAME) + ".type",
    test_controller::TEST_CONTROLLER_TYPE);
  cm_->set_parameter(joint_parameters);
  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);
}

TEST_F(TestControllerManagerSrvs, unload_controller_srv) {
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::UnloadController>(
    "unload_controller");

  auto request = std::make_shared<controller_manager_msgs::srv::UnloadController::Request>();
  request->name = test_controller::TEST_CONTROLLER_NAME;
  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_FALSE(result->ok) << "Controller not loaded: " << request->name;


  auto test_controller = std::make_shared<test_controller::TestController>();
  auto abstract_test_controller = cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_TYPE);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());

  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);
  EXPECT_EQ(0u, cm_->get_loaded_controllers().size());
}