/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include <rclcpp/rclcpp.hpp>

namespace dlio {

    template <typename T>
    void declare_param(rclcpp::Node* node, const std::string& param_name, T& param, const T& default_value) {
        param = node->declare_parameter(param_name, default_value);
    }

}