// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <list>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "std_msgs/msg/u_int32.hpp"

template<typename T>
struct pointer_traits
{
  using reference = T &;
  using const_reference = const T &;
};

template<>
struct pointer_traits<void>
{
};

// For demonstration purposes only, not necessary for allocator_traits
static uint32_t num_allocs = 0;
static uint32_t num_deallocs = 0;
// A very simple custom allocator. Counts calls to allocate and deallocate.
template<typename T = void>
struct MyAllocator : public pointer_traits<T>
{
public:
  using value_type = T;
  using size_type = std::size_t;
  using pointer = T *;
  using const_pointer = const T *;
  using difference_type = typename std::pointer_traits<pointer>::difference_type;

  MyAllocator() noexcept
  {
  }

  ~MyAllocator() noexcept {}

  template<typename U>
  MyAllocator(const MyAllocator<U> &) noexcept
  {
  }

  T * allocate(size_t size, const void * = 0)
  {
    if (size == 0) {
      return nullptr;
    }
    num_allocs++;
    return static_cast<T *>(std::malloc(size * sizeof(T)));
  }

  void deallocate(T * ptr, size_t size)
  {
    (void)size;
    if (!ptr) {
      return;
    }
    num_deallocs++;
    std::free(ptr);
  }

  template<typename U, typename ... Args,
  typename std::enable_if<!std::is_const<U>::value>::type * = nullptr>
  void
  construct(U * ptr, Args && ... args)
  {
    ::new(ptr)U(std::forward<Args>(args) ...);
  }

  template<typename U>
  void
  destroy(U * ptr)
  {
    ptr->~U();
  }

  template<typename U>
  struct rebind
  {
    typedef MyAllocator<U> other;
  };
};

template<typename T, typename U>
constexpr bool operator==(const MyAllocator<T> &,
  const MyAllocator<U> &) noexcept
{
  return true;
}

template<typename T, typename U>
constexpr bool operator!=(const MyAllocator<T> &,
  const MyAllocator<U> &) noexcept
{
  return false;
}

// Override global new and delete to count calls during execution.

static bool is_running = false;
static uint32_t global_runtime_allocs = 0;
static uint32_t global_runtime_deallocs = 0;

void * operator new(std::size_t size)
{
  if (is_running) {
    global_runtime_allocs++;
  }
  return std::malloc(size);
}


void operator delete(void * ptr) noexcept
{
  if (ptr != nullptr) {
    if (is_running) {
      global_runtime_deallocs++;
    }
    std::free(ptr);
    ptr = nullptr;
  }
}

int main(int argc, char ** argv)
{
  using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node;

  std::list<std::string> keys = {"intra", "intraprocess", "intra-process", "intra_process"};
  bool intra_process = false;

  if (argc > 1) {
    for (auto & key : keys) {
      if (std::string(argv[1]) == key) {
        intra_process = true;
        break;
      }
    }
  }

  if (intra_process) {
    printf("Intra-process pipeline is ON.\n");
    auto context = rclcpp::contexts::default_context::get_global_default_context();
    auto ipm_state =
      std::make_shared<rclcpp::intra_process_manager::IntraProcessManagerImpl<MyAllocator<>>>();
    // Constructs the intra-process manager with a custom allocator.
    context->get_sub_context<rclcpp::intra_process_manager::IntraProcessManager>(ipm_state);
    node = rclcpp::Node::make_shared("allocator_example", true);
  } else {
    printf("Intra-process pipeline is OFF.\n");
    node = rclcpp::Node::make_shared("allocator_example", false);
  }

  uint32_t counter = 0;
  auto callback = [&counter](std_msgs::msg::UInt32::SharedPtr msg) -> void
    {
      (void)msg;
      ++counter;
    };

  // Create a custom allocator and pass the allocator to the publisher and subscriber.
  auto alloc = std::make_shared<MyAllocator<void>>();
  auto publisher = node->create_publisher<std_msgs::msg::UInt32>("allocator_example", 10, alloc);
  auto msg_mem_strat =
    std::make_shared<rclcpp::message_memory_strategy::MessageMemoryStrategy<std_msgs::msg::UInt32,
    MyAllocator<>>>(alloc);
  auto subscriber = node->create_subscription<std_msgs::msg::UInt32>(
    "allocator_example", 10, callback, nullptr, false, msg_mem_strat, alloc);

  // Create a MemoryStrategy, which handles the allocations made by the Executor during the
  // execution path, and inject the MemoryStrategy into the Executor.
  std::shared_ptr<rclcpp::memory_strategy::MemoryStrategy> memory_strategy =
    std::make_shared<AllocatorMemoryStrategy<MyAllocator<>>>(alloc);

  rclcpp::executor::ExecutorArgs args;
  args.memory_strategy = memory_strategy;
  rclcpp::executors::SingleThreadedExecutor executor(args);

  // Add our node to the executor.
  executor.add_node(node);

  // Create a message with the custom allocator, so that when the Executor deallocates the
  // message on the execution path, it will use the custom deallocate.
  auto msg = std::allocate_shared<std_msgs::msg::UInt32>(*alloc.get());

  rclcpp::utilities::sleep_for(std::chrono::milliseconds(1));
  is_running = true;

  uint32_t i = 0;
  while (rclcpp::ok()) {
    msg->data = i;
    ++i;
    publisher->publish(msg);
    rclcpp::utilities::sleep_for(std::chrono::milliseconds(1));
    executor.spin_some();
  }
  is_running = false;

  uint32_t final_global_allocs = global_runtime_allocs;
  uint32_t final_global_deallocs = global_runtime_deallocs;
  printf("Global new was called %u times during spin\n", final_global_allocs);
  printf("Global delete was called %u times during spin\n", final_global_deallocs);

  printf("Allocator new was called %u times during spin\n", num_allocs);
  printf("Allocator delete was called %u times during spin\n", num_deallocs);
}
