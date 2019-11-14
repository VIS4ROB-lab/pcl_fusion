/*
* Copyright (c) 2018, Vision for Robotics Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Vision for Robotics Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

/*
 * parameter-reader.cpp
 * @brief Header file for the ParameterReader class.
 * @author: Marco Karrer
 * Created on: Aug 14, 2018
 */

#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "parameters.hpp"

/// \pgbe Main namespace of this package
namespace pgbe {

class ParameterReader {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  /// \brief Class constructor
  /// @param nh The ros node handle.
  /// @param num_agents The number of agents participating. 
  ParameterReader(ros::NodeHandle& nh,
                  const size_t num_agents);
  ~ParameterReader(); 

  /// \brief Get the parameters
  /// @param params The parameters.
  /// @return Whether or not the operation was successful.
  bool getParameters(SystemParameters& params);

protected:
  /// \brief Read the parameters.
  /// @param params The parameters.
  /// @return Whether or not the operation was successful.
  bool readParameters(SystemParameters& params);

  ros::NodeHandle* nh_;
  const size_t num_agents_;

  SystemParameters parameters_;
  bool read_parameters_;
};

} // namespace pgbe
