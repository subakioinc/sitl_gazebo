/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Anton Matosov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H

#include <string>
#include <random>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "Wind.pb.h"

// 하는 일
// - 바람의 세기 및 방향에 따라 모델에 작용 되는 상황 시뮬레이션을 위한 플러그인 

namespace gazebo {
// Default values
// 바람의 풍속 방향등 적용 되는 인자들 초기화 
// - wind : 
//  - 평균풍속, 최대풍속, 풍속의 가변값
// - gust : 
//  - 평균풍속, 최대풍속, 풍속의 가변값
// - direction :
//  - 돌풍의 평균풍향, 바람의 평균풍향
// - 
static const std::string kDefaultNamespace = "";
static const std::string kDefaultFrameId = "world";

static constexpr double kDefaultWindVelocityMean = 0.0;
static constexpr double kDefaultWindVelocityMax = 100.0;
static constexpr double kDefaultWindVelocityVariance = 0.0;
static constexpr double kDefaultWindGustVelocityMean = 0.0;
static constexpr double kDefaultWindGustVelocityMax = 10.0;
static constexpr double kDefaultWindGustVelocityVariance = 0.0;

// 돌풍의 시작 시간 
// 돌풍의 지속 시간 
static constexpr double kDefaultWindGustStart = 10.0;
static constexpr double kDefaultWindGustDuration = 0.0;

static const ignition::math::Vector3d kDefaultWindDirectionMean = ignition::math::Vector3d(1, 0, 0);
static const ignition::math::Vector3d kDefaultWindGustDirectionMean = ignition::math::Vector3d(0, 1, 0);
static constexpr double kDefaultWindDirectionVariance = 0.0;
static constexpr double kDefaultWindGustDirectionVariance = 0.0;

/// \brief This gazebo plugin simulates wind acting on a model.
/// 이 gazebo plugin은 모델에 작용하는 바람을 시뮬레이션 한다 
class GazeboWindPlugin : public WorldPlugin {
// windplugin 초기화 
 public:
  GazeboWindPlugin()
      : WorldPlugin(),
        namespace_(kDefaultNamespace),
        wind_pub_topic_("world_wind"),
        wind_velocity_mean_(kDefaultWindVelocityMean),
        wind_velocity_max_(kDefaultWindVelocityMax),
        wind_velocity_variance_(kDefaultWindVelocityVariance),
        wind_gust_velocity_mean_(kDefaultWindGustVelocityMean),
        wind_gust_velocity_max_(kDefaultWindGustVelocityMax),
        wind_gust_velocity_variance_(kDefaultWindGustVelocityVariance),
        wind_direction_mean_(kDefaultWindDirectionMean),
        wind_direction_variance_(kDefaultWindDirectionVariance),
        wind_gust_direction_mean_(kDefaultWindGustDirectionMean),
        wind_gust_direction_variance_(kDefaultWindGustDirectionVariance),
        frame_id_(kDefaultFrameId),
        pub_interval_(0.5),
        node_handle_(NULL) {}

  virtual ~GazeboWindPlugin();

 protected:
  /// \brief Load the plugin.
  /// \param[in] _model Pointer to the model that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  /// 플러그인을 로드하는 함수
  /// 첫번째 인자 : 이 플러그인을 로드하는 모델을 포인터로 갖는다 (부모 모델)
  /// 두번째 인자 : 이 플러그인을 설명하는 sdf요소들 
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf);

  /// \brief Called when the world is updated.
  /// \param[in] _info Update timing information.
  /// world가 업데이트 될때 호출 되는 콜백 함수 
  /// 인자 : 업데이트 되는 시간 정보 
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  /// \brief Pointer to the update event connection.
  /// 업데이트 이벤트와 연결하는 포인터 
  event::ConnectionPtr update_connection_;

  physics::WorldPtr world_;

  std::string namespace_;

  std::string frame_id_;
  std::string wind_pub_topic_;

  double wind_velocity_mean_;
  double wind_velocity_max_;
  double wind_velocity_variance_;
  double wind_gust_velocity_mean_;
  double wind_gust_velocity_max_;
  double wind_gust_velocity_variance_;
  double pub_interval_;
  std::default_random_engine wind_velocity_generator_;
  std::normal_distribution<double> wind_velocity_distribution_;
  std::default_random_engine wind_gust_velocity_generator_;
  std::normal_distribution<double> wind_gust_velocity_distribution_;

  ignition::math::Vector3d xyz_offset_;
  ignition::math::Vector3d wind_direction_mean_;
  ignition::math::Vector3d wind_gust_direction_mean_;
  double wind_direction_variance_;
  double wind_gust_direction_variance_;
  std::default_random_engine wind_direction_generator_;
  std::normal_distribution<double> wind_direction_distribution_X_;
  std::normal_distribution<double> wind_direction_distribution_Y_;
  std::normal_distribution<double> wind_direction_distribution_Z_;
  std::default_random_engine wind_gust_direction_generator_;
  std::normal_distribution<double> wind_gust_direction_distribution_X_;
  std::normal_distribution<double> wind_gust_direction_distribution_Y_;
  std::normal_distribution<double> wind_gust_direction_distribution_Z_;

  common::Time wind_gust_end_;
  common::Time wind_gust_start_;
  common::Time last_time_;

// gazebo 내의 통신을 위한 handle 및 포인터 
  transport::NodePtr node_handle_;
  transport::PublisherPtr wind_pub_;

  physics_msgs::msgs::Wind wind_msg;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H
