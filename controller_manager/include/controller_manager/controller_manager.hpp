// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
#define CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/controller_interface_base.hpp"

#include "controller_manager/controller_spec.hpp"
#include "controller_manager_msgs/msg/controller_manager_activity.hpp"
#include "controller_manager_msgs/srv/configure_controller.hpp"
#include "controller_manager_msgs/srv/list_controller_types.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "controller_manager_msgs/srv/list_hardware_interfaces.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/reload_controller_libraries.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/unload_controller.hpp"

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "hardware_interface/resource_manager.hpp"

#include "pluginlib/class_loader.hpp"

#include "rclcpp/executor.hpp"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"

namespace controller_manager
{
class ParamListener;
class Params;
using ControllersListIterator = std::vector<controller_manager::ControllerSpec>::const_iterator;

rclcpp::NodeOptions get_cm_node_options();

class ControllerManager : public rclcpp::Node
{
public:
  static constexpr bool kWaitForAllResources = false;
  static constexpr auto kInfiniteTimeout = 0;

  ControllerManager(
    std::unique_ptr<hardware_interface::ResourceManager> resource_manager,
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & node_namespace = "",
    const rclcpp::NodeOptions & options = get_cm_node_options());

  ControllerManager(
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & node_namespace = "",
    const rclcpp::NodeOptions & options = get_cm_node_options());

  ControllerManager(
    std::shared_ptr<rclcpp::Executor> executor, const std::string & urdf,
    bool activate_all_hw_components, const std::string & manager_node_name = "controller_manager",
    const std::string & node_namespace = "",
    const rclcpp::NodeOptions & options = get_cm_node_options());

  virtual ~ControllerManager();

  /// Shutdown all controllers in the controller manager.
  /**
   * \return true if all controllers are successfully shutdown, false otherwise.
   */
  bool shutdown_controllers();

  void robot_description_callback(const std_msgs::msg::String & msg);

  void init_resource_manager(const std::string & robot_description);

  controller_interface::ControllerInterfaceBaseSharedPtr load_controller(
    const std::string & controller_name, const std::string & controller_type);

  /// load_controller loads a controller by name, the type must be defined in the parameter server.
  /**
   * \param[in] controller_name as a string.
   * \return controller
   * \see Documentation in controller_manager_msgs/LoadController.srv
   */
  controller_interface::ControllerInterfaceBaseSharedPtr load_controller(
    const std::string & controller_name);

  controller_interface::return_type unload_controller(const std::string & controller_name);

  std::vector<ControllerSpec> get_loaded_controllers() const;

  template <
    typename T, typename std::enable_if<
                  std::is_convertible<T *, controller_interface::ControllerInterfaceBase *>::value,
                  T>::type * = nullptr>
  controller_interface::ControllerInterfaceBaseSharedPtr add_controller(
    std::shared_ptr<T> controller, const std::string & controller_name,
    const std::string & controller_type)
  {
    ControllerSpec controller_spec;
    controller_spec.c = controller;
    controller_spec.info.name = controller_name;
    controller_spec.info.type = controller_type;
    controller_spec.last_update_cycle_time = std::make_shared<rclcpp::Time>(0);
    return add_controller_impl(controller_spec);
  }

  controller_interface::ControllerInterfaceBaseSharedPtr add_controller(
    const ControllerSpec & controller_spec)
  {
    return add_controller_impl(controller_spec);
  }

  /// configure_controller Configure controller by name calling their "configure" method.
  /**
   * \param[in] controller_name as a string.
   * \return configure controller response
   * \see Documentation in controller_manager_msgs/ConfigureController.srv
   */
  controller_interface::return_type configure_controller(const std::string & controller_name);

  /// switch_controller Deactivates some controllers and activates others.
  /**
   * \param[in] activate_controllers is a list of controllers to activate.
   * \param[in] deactivate_controllers is a list of controllers to deactivate.
   * \param[in] set level of strictness (BEST_EFFORT or STRICT)
   * \param[in] activate_asap flag to activate controllers as soon as possible.
   * \param[in] timeout to wait for the controllers to be switched.
   * \see Documentation in controller_manager_msgs/SwitchController.srv
   */
  controller_interface::return_type switch_controller(
    const std::vector<std::string> & activate_controllers,
    const std::vector<std::string> & deactivate_controllers, int strictness,
    bool activate_asap = kWaitForAllResources,
    const rclcpp::Duration & timeout = rclcpp::Duration::from_nanoseconds(kInfiniteTimeout));

  /// switch_controller Deactivates some controllers and activates others.
  /**
   * \param[in] activate_controllers is a list of controllers to activate.
   * \param[in] deactivate_controllers is a list of controllers to deactivate.
   * \param[in] set level of strictness (BEST_EFFORT or STRICT)
   * \param[in] activate_asap flag to activate controllers as soon as possible.
   * \param[in] timeout to wait for the controllers to be switched.
   * \param[out] message describing the result of the switch.
   * \see Documentation in controller_manager_msgs/SwitchController.srv
   */
  controller_interface::return_type switch_controller_cb(
    const std::vector<std::string> & activate_controllers,
    const std::vector<std::string> & deactivate_controllers, int strictness, bool activate_asap,
    const rclcpp::Duration & timeout, std::string & message);

  /// Read values to state interfaces.
  /**
   * Read current values from hardware to state interfaces.
   * **The method called in the (real-time) control loop.**
   *
   * \param[in]  time    The time at the start of this control loop iteration
   * \param[in]  period  The measured period of the last control loop iteration
   */
  void read(const rclcpp::Time & time, const rclcpp::Duration & period);

  /// Run update on controllers
  /**
   * Call update of all controllers.
   * **The method called in the (real-time) control loop.**
   *
   * \param[in]  time    The time at the start of this control loop iteration
   * \param[in]  period  The measured period of the last control loop iteration
   */
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period);

  /// Write values from command interfaces.
  /**
   * Write values from command interface into hardware.
   * **The method called in the (real-time) control loop.**
   *
   * \param[in]  time    The time at the start of this control loop iteration
   * \param[in]  period  The measured period of the last control loop iteration
   */
  void write(const rclcpp::Time & time, const rclcpp::Duration & period);

  /// Deterministic (real-time safe) callback group, e.g., update function.
  /**
   * Deterministic (real-time safe) callback group for the update function. Default behavior
   * is read hardware, update controller and finally write new values to the hardware.
   */
  // TODO(anyone): Due to issues with the MutliThreadedExecutor, this control loop does not rely on
  // the executor (see issue #260).
  // rclcpp::CallbackGroup::SharedPtr deterministic_callback_group_;

  /// Interface for external components to check if Resource Manager is initialized.
  /**
   * Checks if components in Resource Manager are loaded and initialized.
   * \returns true if they are initialized, false otherwise.
   */
  bool is_resource_manager_initialized() const
  {
    return resource_manager_ && resource_manager_->are_components_initialized();
  }

  /// Update rate of the main control loop in the controller manager.
  /**
   * Update rate of the main control loop in the controller manager.
   * The method is used for per-controller update rate support.
   *
   * \returns update rate of the controller manager.
   */
  unsigned int get_update_rate() const;

  /// Get the trigger clock of the controller manager.
  /**
   * Get the trigger clock of the controller manager.
   * The method is used to get the clock that is used for triggering the controllers and the
   * hardware components.
   *
   * @note When the use_sim_time parameter is set to true, the clock will be the ROS clock.
   * Otherwise, the clock will be the Steady Clock.
   *
   * \returns trigger clock of the controller manager.
   */
  rclcpp::Clock::SharedPtr get_trigger_clock() const;

protected:
  void init_services();

  controller_interface::ControllerInterfaceBaseSharedPtr add_controller_impl(
    const ControllerSpec & controller);

  void manage_switch();

  /// Deactivate chosen controllers from real-time controller list.
  /**
   * Deactivate controllers with names \p controllers_to_deactivate from list \p rt_controller_list.
   * The controller list will be iterated as many times as there are controller names.
   *
   * \param[in] rt_controller_list controllers in the real-time list.
   * \param[in] controllers_to_deactivate names of the controller that have to be deactivated.
   */
  void deactivate_controllers(
    const std::vector<ControllerSpec> & rt_controller_list,
    const std::vector<std::string> controllers_to_deactivate);

  /**
   * Switch chained mode for all the controllers with respect to the following cases:
   * - a preceding controller is getting activated --> switch controller to chained mode;
   * - all preceding controllers are deactivated --> switch controller from chained mode.
   *
   * \param[in] chained_mode_switch_list list of controller to switch chained mode.
   * \param[in] to_chained_mode flag if controller should be switched *to* or *from* chained mode.
   */
  void switch_chained_mode(
    const std::vector<std::string> & chained_mode_switch_list, bool to_chained_mode);

  /// Activate chosen controllers from real-time controller list.
  /**
   * Activate controllers with names \p controllers_to_activate from list \p rt_controller_list.
   * The controller list will be iterated as many times as there are controller names.
   *
   * \param[in] rt_controller_list controllers in the real-time list.
   * \param[in] controllers_to_activate names of the controller that have to be activated.
   */
  void activate_controllers(
    const std::vector<ControllerSpec> & rt_controller_list,
    const std::vector<std::string> controllers_to_activate);

  /// Activate chosen controllers from real-time controller list.
  /**
   * Activate controllers with names \p controllers_to_activate from list \p rt_controller_list.
   * The controller list will be iterated as many times as there are controller names.
   *
   * *NOTE*: There is currently not difference to `activate_controllers` method.
   * Check https://github.com/ros-controls/ros2_control/issues/263 for more information.
   *
   * \param[in] rt_controller_list controllers in the real-time list.
   * \param[in] controllers_to_activate names of the controller that have to be activated.
   */
  void activate_controllers_asap(
    const std::vector<ControllerSpec> & rt_controller_list,
    const std::vector<std::string> controllers_to_activate);

  void list_controllers_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> response);

  void list_hardware_interfaces_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListHardwareInterfaces::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListHardwareInterfaces::Response> response);

  void load_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::LoadController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::LoadController::Response> response);

  void configure_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Response> response);

  void reload_controller_libraries_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Response> response);

  void switch_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> response);

  void unload_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::UnloadController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::UnloadController::Response> response);

  void list_controller_types_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Response> response);

  void list_hardware_components_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListHardwareComponents::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListHardwareComponents::Response> response);

  void set_hardware_component_state_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::SetHardwareComponentState::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::SetHardwareComponentState::Response> response);

  // Per controller update rate support
  unsigned int update_loop_counter_ = 0;
  unsigned int update_rate_;
  std::vector<std::vector<std::string>> chained_controllers_configuration_;

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_;

private:
  std::vector<std::string> get_controller_names();
  std::pair<std::string, std::string> split_command_interface(
    const std::string & command_interface);
  void init_controller_manager();

  void initialize_parameters();

  /**
   * Call cleanup to change the given controller lifecycle node to the unconfigured state.
   *
   * \param[in] controller controller to be shutdown.
   */
  controller_interface::return_type cleanup_controller(
    const controller_manager::ControllerSpec & controller);

  /**
   * Call shutdown to change the given controller lifecycle node to the finalized state.
   *
   * \param[in] controller controller to be shutdown.
   */
  void shutdown_controller(const controller_manager::ControllerSpec & controller) const;

  /**
   * Clear request lists used when switching controllers. The lists are shared between "callback"
   * and "control loop" threads.
   */
  void clear_requests();

  /**
   * Perform hardware command mode change for the given list of controllers to activate and
   * deactivate.
   * \param[in] rt_controller_list list of controllers in the real-time list.
   * \param[in] activate_controllers_list list of controllers to activate.
   * \param[in] deactivate_controllers_list list of controllers to deactivate.
   * \param[in] rt_cycle_name name of the real-time cycle.
   * \note This method is meant to be used only in the real-time control loops (`read`, `update` and
   * `write`).
   */
  void perform_hardware_command_mode_change(
    const std::vector<ControllerSpec> & rt_controller_list,
    const std::vector<std::string> & activate_controllers_list,
    const std::vector<std::string> & deactivate_controllers_list,
    const std::string & rt_cycle_name);

  /**
   * If a controller is deactivated all following controllers (if any exist) should be switched
   * 'from' the chained mode.
   *
   * \param[in] controllers list with controllers.
   */
  void propagate_deactivation_of_chained_mode(const std::vector<ControllerSpec> & controllers);

  /// Check if all the following controllers will be in active state and in the chained mode
  /// after controllers' switch.
  /**
   * Check recursively that all following controllers of the @controller_it
   * - are already active,
   * - will not be deactivated,
   * - or will be activated.
   * The following controllers are added to the request to switch in the chained mode or removed
   * from the request to switch from the chained mode.
   *
   * For each controller the whole chain of following controllers is checked.
   *
   * NOTE: The automatically adding of following controller into activate list is not implemented
   * yet.
   *
   * \param[in] controllers list with controllers.
   * \param[in] strictness if value is equal "MANIPULATE_CONTROLLERS_CHAIN" then all following
   * controllers will be automatically added to the activate request list if they are not in the
   * deactivate request.
   * \param[in] controller_it iterator to the controller for which the following controllers are
   * \param[out] message describing the result of the check.
   * checked.
   *
   * \returns return_type::OK if all following controllers pass the checks, otherwise
   * return_type::ERROR.
   */
  controller_interface::return_type check_following_controllers_for_activate(
    const std::vector<ControllerSpec> & controllers, int strictness,
    const ControllersListIterator controller_it, std::string & message);

  /// Check if all the preceding controllers will be in inactive state after controllers' switch.
  /**
   * Check that all preceding controllers of the @controller_it
   * - are inactive,
   * - will be deactivated,
   * - and will not be activated.
   *
   * NOTE: The automatically adding of preceding controllers into deactivate list is not implemented
   * yet.
   *
   * \param[in] controllers list with controllers.
   * \param[in] strictness if value is equal "MANIPULATE_CONTROLLERS_CHAIN" then all preceding
   * controllers will be automatically added to the deactivate request list.
   * \param[in] controller_it iterator to the controller for which the preceding controllers are
   * checked.
   * \param[out] message describing the result of the check.
   *
   * \returns return_type::OK if all preceding controllers pass the checks, otherwise
   * return_type::ERROR.
   */
  controller_interface::return_type check_preceding_controllers_for_deactivate(
    const std::vector<ControllerSpec> & controllers, int strictness,
    const ControllersListIterator controller_it, std::string & message);

  /// Checks if the fallback controllers of the given controllers are in the right
  /// state, so they can be activated immediately
  /**
   * \param[in] controllers is a list of controllers to activate.
   * \param[in] controller_it is the iterator pointing to the controller to be activated.
   * \param[out] message describing the result of the check.
   * \return return_type::OK if all fallback controllers are in the right state, otherwise
   * return_type::ERROR.
   */
  controller_interface::return_type check_fallback_controllers_state_pre_activation(
    const std::vector<ControllerSpec> & controllers, const ControllersListIterator controller_it,
    std::string & message);

  /**
   * Checks that all the interfaces required by the controller are available to activate.
   *
   * \param[in] controllers list with controllers.
   * \param[in] activation_list list with controllers to activate.
   * \param[out] message describing the result of the check.
   * \return return_type::OK if all interfaces are available, otherwise return_type::ERROR.
   */
  controller_interface::return_type check_for_interfaces_availability_to_activate(
    const std::vector<ControllerSpec> & controllers, const std::vector<std::string> activation_list,
    std::string & message);

  /**
   * @brief Inserts a controller into an ordered list based on dependencies to compute the
   * controller chain.
   *
   * This method computes the controller chain by inserting the provided controller name into an
   * ordered list of controllers based on dependencies. It ensures that controllers are inserted in
   * the correct order so that dependencies are satisfied.
   *
   * @param ctrl_name The name of the controller to be inserted into the chain.
   * @param controller_iterator An iterator pointing to the position in the ordered list where the
   * controller should be inserted.
   * @param append_to_controller Flag indicating whether the controller should be appended or
   * prepended to the parsed iterator.
   * @note The specification of controller dependencies is in the ControllerChainSpec,
   * containing information about following and preceding controllers. This struct should include
   * the neighboring controllers with their relationships to the provided controller.
   * `following_controllers` specify controllers that come after the provided controller.
   * `preceding_controllers` specify controllers that come before the provided controller.
   */
  void update_list_with_controller_chain(
    const std::string & ctrl_name, std::vector<std::string>::iterator controller_iterator,
    bool append_to_controller);

  /**
   * @brief Method to publish the state of the controller manager.
   * The state includes the list of controllers and the list of hardware interfaces along with
   * their states.
   */
  void publish_activity();

  void controller_activity_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper & stat);

  void hardware_components_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper & stat);

  void controller_manager_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief determine_controller_node_options - A method that retrieves the controller defined node
   * options and adapts them, based on if there is a params file to be loaded or the use_sim_time
   * needs to be set
   * @param controller - controller info
   * @return The node options that will be set to the controller LifeCycleNode
   */
  rclcpp::NodeOptions determine_controller_node_options(const ControllerSpec & controller) const;

  /**
   * @brief cleanup_controller_exported_interfaces - A method that cleans up the exported interfaces
   * of a chainable controller
   * @param controller - controller info
   */
  void cleanup_controller_exported_interfaces(const ControllerSpec & controller);

  std::shared_ptr<controller_manager::ParamListener> cm_param_listener_;
  std::shared_ptr<controller_manager::Params> params_;
  diagnostic_updater::Updater diagnostics_updater_;

  std::shared_ptr<rclcpp::Executor> executor_;

  std::shared_ptr<pluginlib::ClassLoader<controller_interface::ControllerInterface>> loader_;
  std::shared_ptr<pluginlib::ClassLoader<controller_interface::ChainableControllerInterface>>
    chainable_loader_;

  /// Best effort (non real-time safe) callback group, e.g., service callbacks.
  /**
   * Best effort (non real-time safe) callback group for callbacks that can possibly break
   * real-time requirements, for example, service callbacks.
   */
  rclcpp::CallbackGroup::SharedPtr best_effort_callback_group_;

  /**
   * The RTControllerListWrapper class wraps a double-buffered list of controllers
   * to avoid needing to lock the real-time thread when switching controllers in
   * the non-real-time thread.
   *
   * There's always an "updated" list and an "outdated" one
   * There's always an "used by rt" list and an "unused by rt" list
   *
   * The updated state changes on the switch_updated_list()
   * The rt usage state changes on the update_and_get_used_by_rt_list()
   */
  class RTControllerListWrapper
  {
    // *INDENT-OFF*
  public:
    // *INDENT-ON*
    /// update_and_get_used_by_rt_list Makes the "updated" list the "used by rt" list
    /**
     * \warning Should only be called by the RT thread, no one should modify the
     * updated list while it's being used
     * \return reference to the updated list
     */
    std::vector<ControllerSpec> & update_and_get_used_by_rt_list();

    /**
     * get_unused_list Waits until the "outdated" and "unused by rt"
     * lists match and returns a reference to it
     * This referenced list can be modified safely until switch_updated_controller_list()
     * is called, at this point the RT thread may start using it at any time
     * \param[in] guard Guard needed to make sure the caller is the only one accessing the unused by
     * rt list
     */
    std::vector<ControllerSpec> & get_unused_list(
      const std::lock_guard<std::recursive_mutex> & guard);

    /// get_updated_list Returns a const reference to the most updated list.
    /**
     * \warning May or may not being used by the realtime thread, read-only reference for safety
     * \param[in] guard Guard needed to make sure the caller is the only one accessing the unused by
     * rt list
     */
    const std::vector<ControllerSpec> & get_updated_list(
      const std::lock_guard<std::recursive_mutex> & guard) const;

    /**
     * switch_updated_list Switches the "updated" and "outdated" lists, and waits
     *  until the RT thread is using the new "updated" list.
     * \param[in] guard Guard needed to make sure the caller is the only one accessing the unused by
     * rt list
     */
    void switch_updated_list(const std::lock_guard<std::recursive_mutex> & guard);

    /// A method to register a callback to be called when the list is switched
    /**
     * \param[in] callback Callback to be called when the list is switched
     */
    void set_on_switch_callback(std::function<void()> callback);

    // Mutex protecting the controllers list
    // must be acquired before using any list other than the "used by rt"
    mutable std::recursive_mutex controllers_lock_;

    // *INDENT-OFF*
  private:
    // *INDENT-ON*
    /// get_other_list get the list not pointed by index
    /**
     * \param[in] index int
     */
    int get_other_list(int index) const;

    void wait_until_rt_not_using(
      int index, std::chrono::microseconds sleep_delay = std::chrono::microseconds(200)) const;

    std::vector<ControllerSpec> controllers_lists_[2];
    /// The index of the controller list with the most updated information
    int updated_controllers_index_ = 0;
    /// The index of the controllers list being used in the real-time thread.
    int used_by_realtime_controllers_index_ = -1;
    /// The callback to be called when the list is switched
    std::function<void()> on_switch_callback_ = nullptr;
  };

  rclcpp::Clock::SharedPtr trigger_clock_ = nullptr;
  std::unique_ptr<rclcpp::PreShutdownCallbackHandle> preshutdown_cb_handle_{nullptr};
  RTControllerListWrapper rt_controllers_wrapper_;
  std::unordered_map<std::string, ControllerChainSpec> controller_chain_spec_;
  std::vector<std::string> ordered_controllers_names_;
  /// mutex copied from ROS1 Control, protects service callbacks
  /// not needed if we're guaranteed that the callbacks don't come from multiple threads
  std::mutex services_lock_;
  rclcpp::Publisher<controller_manager_msgs::msg::ControllerManagerActivity>::SharedPtr
    controller_manager_activity_publisher_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllers>::SharedPtr
    list_controllers_service_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllerTypes>::SharedPtr
    list_controller_types_service_;
  rclcpp::Service<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::ConfigureController>::SharedPtr
    configure_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::ReloadControllerLibraries>::SharedPtr
    reload_controller_libraries_service_;
  rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::UnloadController>::SharedPtr
    unload_controller_service_;

  rclcpp::Service<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr
    list_hardware_components_service_;
  rclcpp::Service<controller_manager_msgs::srv::ListHardwareInterfaces>::SharedPtr
    list_hardware_interfaces_service_;
  rclcpp::Service<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr
    set_hardware_component_state_service_;

  std::vector<std::string> activate_request_, deactivate_request_;
  std::vector<std::string> to_chained_mode_request_, from_chained_mode_request_;
  std::vector<std::string> activate_command_interface_request_,
    deactivate_command_interface_request_;

  std::map<std::string, std::vector<std::string>> controller_chained_reference_interfaces_cache_;
  std::map<std::string, std::vector<std::string>> controller_chained_state_interfaces_cache_;

  rclcpp::NodeOptions cm_node_options_;
  std::string robot_description_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
  rclcpp::TimerBase::SharedPtr robot_description_notification_timer_;
  bool enforce_command_limits_;

  controller_manager::MovingAverageStatistics periodicity_stats_;

  struct SwitchParams
  {
    void reset()
    {
      do_switch = false;
      started = false;
      strictness = 0;
      activate_asap = false;
    }

    bool do_switch;
    bool started;

    // Switch options
    int strictness;
    bool activate_asap;
    std::chrono::nanoseconds timeout;

    // conditional variable and mutex to wait for the switch to complete
    std::condition_variable cv;
    std::mutex mutex;
  };

  SwitchParams switch_params_;

  struct RTBufferVariables
  {
    RTBufferVariables()
    {
      deactivate_controllers_list.reserve(1000);
      activate_controllers_using_interfaces_list.reserve(1000);
      fallback_controllers_list.reserve(1000);
      interfaces_to_start.reserve(1000);
      interfaces_to_stop.reserve(1000);
      concatenated_string.reserve(5000);
    }

    const std::string & get_concatenated_string(
      const std::vector<std::string> & strings, bool clear_string = true)
    {
      if (clear_string)
      {
        concatenated_string.clear();
      }
      for (const auto & str : strings)
      {
        concatenated_string.append(str);
        concatenated_string.append(" ");
      }
      return concatenated_string;
    }

    std::vector<std::string> deactivate_controllers_list;
    std::vector<std::string> activate_controllers_using_interfaces_list;
    std::vector<std::string> fallback_controllers_list;
    std::vector<std::string> interfaces_to_start;
    std::vector<std::string> interfaces_to_stop;
    std::string concatenated_string;
  };
  RTBufferVariables rt_buffer_;
};

}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
