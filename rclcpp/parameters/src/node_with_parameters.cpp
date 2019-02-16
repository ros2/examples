// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/unused_parameters_checker.hpp"

namespace rclcpp
{

template<typename ParameterType>
class Immutable
{
public:
  using type = ParameterType;
  using is_mutable = std::false_type;

  Immutable(
    rclcpp::node_interfaces::NodeParametersInterface * node_parameter_interface,
    const std::string & name)
  : node_parameter_interface_(node_parameter_interface),
    name_(name)
  {}

  const std::string &
  get_name() const
  {
    return name_;
  }

  auto
  get() const
  {
    return node_parameter_interface_->get_parameter(name_).template get_value<ParameterType>();
  }

  bool
  get(rclcpp::Parameter & parameter) const
  {
    return node_parameter_interface_->get_parameter(name_, parameter);
  }

protected:
  rclcpp::node_interfaces::NodeParametersInterface *
  get_node_parameters_interface() const
  {
    return node_parameter_interface_;
  }

private:
  rclcpp::node_interfaces::NodeParametersInterface * node_parameter_interface_;
  const std::string name_;
};

template<typename ParameterType>
class Mutable : public Immutable<ParameterType>
{
public:
  using is_mutable = std::true_type;

  Mutable(
    rclcpp::node_interfaces::NodeParametersInterface * node_parameter_interface,
    const std::string & name)
  : Immutable<ParameterType>(node_parameter_interface, name)
  {}

  rcl_interfaces::msg::SetParametersResult
  set(const ParameterType & new_value, const std::nothrow_t &)
  {
    return this->get_node_parameters_interface()->set_parameters_atomically({
      rclcpp::Parameter(this->get_name(), new_value),
    });
  }

  template<typename SetParameterType>
  rcl_interfaces::msg::SetParametersResult
  set(const SetParameterType & new_value, const std::nothrow_t &)
  {
    static_assert(
      rclcpp::ParameterTypeHelper<ParameterType>::parameter_type ==
        rclcpp::ParameterTypeHelper<SetParameterType>::parameter_type,
      "incompatible type for declared parameter");
    return this->get_node_parameters_interface()->set_parameters_atomically({
      rclcpp::Parameter(this->get_name(), new_value),
    });
  }

  template<typename SetParameterType>
  void
  set(const SetParameterType & new_value)
  {
    const std::string & name = this->get_name();
    auto result = this->set(new_value, std::nothrow);
    if (!result.successful) {
      throw std::runtime_error("failed to set parameter '" + name + "': " + result.reason);
    }
  }
};

struct ParameterOptional
{
  using external_initialization_is_required = std::false_type;
};

struct ParameterRequired
{
  using external_initialization_is_required = std::true_type;
};

rclcpp::node_interfaces::NodeParametersInterface *
get_node_parameter_interface(rclcpp::node_interfaces::NodeParametersInterface * node)
{
  return node;
}

rclcpp::node_interfaces::NodeParametersInterface *
get_node_parameter_interface(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node)
{
  return node.get();
}

rclcpp::node_interfaces::NodeParametersInterface *
get_node_parameter_interface(rclcpp::Node * node)
{
  return node->get_node_parameters_interface().get();
}

rclcpp::node_interfaces::NodeParametersInterface *
get_node_parameter_interface(rclcpp::Node::SharedPtr node)
{
  return node->get_node_parameters_interface().get();
}

template<
  typename ParameterType,
  typename StoragePolicy = Mutable<ParameterType>,
  typename InitializationPolicy = ParameterOptional>
class DescribedParameter : public StoragePolicy, public InitializationPolicy
{
public:
  template<typename NodeType>
  DescribedParameter(
    NodeType * node,
    const std::string & name,
    const ParameterType & default_value)
  : StoragePolicy(get_node_parameter_interface(node), name)
  {
    static_assert(
      std::is_same<ParameterType, typename StoragePolicy::type>::value,
      "expected ParameterType of DescribedParameter to match the type of the StoragePolicy");
    static_assert(
      !InitializationPolicy::external_initialization_is_required::value,
      "a default parameter value is unused when an external initial parameter value is required");
    this->get_node_parameters_interface()->declare_parameter(
      this->get_name(), rclcpp::ParameterValue(default_value), !StoragePolicy::is_mutable::value);
  }

  template<typename NodeType>
  DescribedParameter(
    NodeType node,
    const std::string & name)
  : StoragePolicy(get_node_parameter_interface(node), name)
  {
    static_assert(
      std::is_same<ParameterType, typename StoragePolicy::type>::value,
      "expected ParameterType of DescribedParameter to match the type of the StoragePolicy");
    static_assert(
      std::is_same<ParameterType, rclcpp::Parameter>::value ||
      InitializationPolicy::external_initialization_is_required::value,
      "a parameter with a fixed type no requirement to be set externally may "
      "not be unset, therefore a default value is required");
    this->get_node_parameters_interface()->declare_parameter(
      this->get_name(), rclcpp::ParameterValue(), !StoragePolicy::is_mutable::value);
  }
};

}  // namespace rclcpp

/////////////////////////////////////////////////////////////////////////////////////////////////

class NodeWithParameters : public rclcpp::Node
{
public:
  explicit
  NodeWithParameters(const std::vector<rclcpp::Parameter> & initial_parameter_values)
  : Node(
      "node_with_parameters", "",
      rclcpp::contexts::default_context::get_global_default_context(),
      {}, initial_parameter_values
    ),
    not_set_param_(this, "not_set_param"),
    // ip_address_(*this, "ip_address"),  // compiler error, would be an uninitialized, typed param
    ip_address_(this, "ip_address", "127.0.0.1"),
    port_(this, "port", 80),
    // data_(*this, "data", 3.14)  // compiler error, default value meaningless with external config
    data_(this, "data")
  {
    rclcpp::Parameter not_set_param;
    if (not_set_param_.get(not_set_param)) {
      RCLCPP_ERROR(this->get_logger(), "parameter 'not_set_param' unexpectedly set");
    }

    const std::string & local_ip_address = ip_address_.get();
    (void)local_ip_address;
    auto result = ip_address_.set("foo", std::nothrow);
    if (!result.successful) {
      RCLCPP_INFO(this->get_logger(), ("failed to set 'ip_address': " + result.reason).c_str());
    }
    ip_address_.set("foo");
    // compiler error:
    // ip_address_.set(42);
    //             ^
    //             error: static_assert failed "incompatible type for declared parameter"

    static_assert(decltype(ip_address_)::is_mutable::value, "expected ip address to be mutable");
    static_assert(!decltype(port_)::is_mutable::value, "expected port to be immutable");

    const int & port = port_.get();
    (void)port;
    // compiler error:
    // port_.set(90);
    //      ^
    //      no member named 'set' in 'rclcpp::DescribedParameter<int, rclcpp::Immutable<...>>'

    // This will compile, but will fail at runtime as the parameter is read-only.
    // I recommend deprecating it in the Node API, but leave it in the NodeParametersInterface.
    result = this->set_parameters_atomically({rclcpp::Parameter("port", 90)});
    if (result.successful) {
      RCLCPP_ERROR(this->get_logger(), "able to successfully set 'port' to '90' (unexpected)");
    }

    float data = this->get_parameter("data").get_value<float>();
    (void)data;
    // data_.on_change([](auto data) -> bool {return true;});

    rclcpp::UnusedParametersChecker checker(this);
  }

private:
  // loosely typed parameters, i.e. ones using rclcpp::Parameter, may be unset and may change type
  rclcpp::DescribedParameter<rclcpp::Parameter> not_set_param_;
  // strictly typed parameters, i.e. not using rclcpp::Parameter, may not be unset nor change type
  rclcpp::DescribedParameter<std::string> ip_address_;
  rclcpp::DescribedParameter<int, rclcpp::Immutable<int>> port_;
  rclcpp::DescribedParameter<float, rclcpp::Immutable<float>, rclcpp::ParameterRequired> data_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::vector<rclcpp::Parameter> initial_parameter_values = {
    rclcpp::Parameter("ip_address", "10.0.0.1"),  // optional custom value
    rclcpp::Parameter("data", 3.14),  // runtime failure without, as it is a "required" parameter
  };
  auto node = std::make_shared<NodeWithParameters>(initial_parameter_values);

  rclcpp::spin(node);

  return 0;
}
