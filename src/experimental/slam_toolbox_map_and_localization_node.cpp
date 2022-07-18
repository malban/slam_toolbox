
/**
 * Copyright 2022 Hatchbed L.L.C.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Marc Alban */

#include <memory>
#include "slam_toolbox/experimental/slam_toolbox_map_and_localization.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int stack_size = 40000000;
  {
    auto temp_node = std::make_shared<rclcpp::Node>("slam_toolbox");
    temp_node->declare_parameter("stack_size_to_use");
    if (temp_node->get_parameter("stack_size_to_use", stack_size)) {
      RCLCPP_INFO(temp_node->get_logger(), "Node using stack size %i", (int)stack_size);
      const rlim_t max_stack_size = stack_size;
      struct rlimit stack_limit;
      getrlimit(RLIMIT_STACK, &stack_limit);
      if (stack_limit.rlim_cur < stack_size) {
        stack_limit.rlim_cur = stack_size;
      }
      setrlimit(RLIMIT_STACK, &stack_limit);
    }
  }

  rclcpp::NodeOptions options;
  auto node = std::make_shared<slam_toolbox::MapAndLocalizationSlamToolbox>(options);
  node->configure();
  node->loadPoseGraphByParams();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
