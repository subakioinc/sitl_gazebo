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

#include "gazebo_wind_plugin.h"
#include "common.h"

namespace gazebo {

GazeboWindPlugin::~GazeboWindPlugin() {
  update_connection_->~Connection();
}

void GazeboWindPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf) {
  //초기화 
  // gazebo Load 함수에서는 SDF으로 부터 parameter들을 가지고오고 
  // 계산을 통해 publish하는 동작을 함 
  world_ = world;

  double wind_gust_start = kDefaultWindGustStart;
  double wind_gust_duration = kDefaultWindGustDuration;

  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (sdf->HasElement("xyzOffset"))
    xyz_offset_ = sdf->GetElement("xyzOffset")->Get<ignition::math::Vector3d>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a xyzOffset.\n";

  getSdfParam<std::string>(sdf, "windPubTopic", wind_pub_topic_, wind_pub_topic_);
  double pub_rate = 2.0;
  getSdfParam<double>(sdf, "publishRate", pub_rate, pub_rate); //Wind topic publishing rates
  pub_interval_ = (pub_rate > 0.0) ? 1/pub_rate : 0.0;
  getSdfParam<std::string>(sdf, "frameId", frame_id_, frame_id_);
  // Get the wind params from SDF.
  // SDF파일(world, model)로 부터 wind 파라미터 값을 가지고 옴
  getSdfParam<double>(sdf, "windVelocityMean", wind_velocity_mean_, wind_velocity_mean_);
  getSdfParam<double>(sdf, "windVelocityMax", wind_velocity_max_, wind_velocity_max_);
  getSdfParam<double>(sdf, "windVelocityVariance", wind_velocity_variance_, wind_velocity_variance_);
  getSdfParam<ignition::math::Vector3d>(sdf, "windDirectionMean", wind_direction_mean_, wind_direction_mean_);
  getSdfParam<double>(sdf, "windDirectionVariance", wind_direction_variance_, wind_direction_variance_);
  // Get the wind gust params from SDF.
  // SDF 파일에서 wind gust(돌풍)관련 파라미터를 가지고옴 
  getSdfParam<double>(sdf, "windGustStart", wind_gust_start, wind_gust_start);
  getSdfParam<double>(sdf, "windGustDuration", wind_gust_duration, wind_gust_duration);
  getSdfParam<double>(sdf, "windGustVeloctiyMean", wind_gust_velocity_mean_, wind_gust_velocity_mean_);
  getSdfParam<double>(sdf, "windGustVelocityMax", wind_gust_velocity_max_, wind_gust_velocity_max_);
  getSdfParam<double>(sdf, "windGustVelocityVariance", wind_gust_velocity_variance_, wind_gust_velocity_variance_);
  getSdfParam<ignition::math::Vector3d>(sdf, "windGustDirectionMean", wind_gust_direction_mean_, wind_gust_direction_mean_);
  getSdfParam<double>(sdf, "windGustDirectionVariance", wind_gust_direction_variance_, wind_gust_direction_variance_);

  wind_direction_mean_.Normalize();
  wind_gust_direction_mean_.Normalize();
  wind_gust_start_ = common::Time(wind_gust_start);
  wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);
 
  // Set random wind velocity mean and standard deviation
  // 랜덤한 바람의 평균속도 및 표준편차를 설정 
  wind_velocity_distribution_.param(std::normal_distribution<double>::param_type(wind_velocity_mean_, sqrt(wind_velocity_variance_)));
  // Set random wind direction mean and standard deviation
  // 랜덤한 바람의 평균 풍향 및 표준편차를 설정 
  wind_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.X(), sqrt(wind_direction_variance_)));
  wind_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Y(), sqrt(wind_direction_variance_)));
  wind_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Z(), sqrt(wind_direction_variance_)));
  // Set random wind gust velocity mean and standard deviation
  // 랜덤한 돌풀의 평균속도 및 평균편차를 설정한다 
  wind_gust_velocity_distribution_.param(std::normal_distribution<double>::param_type(wind_gust_velocity_mean_, sqrt(wind_gust_velocity_variance_)));
  // Set random wind gust direction mean and standard deviation
  // 돌풍의 평균방향과 표준편차를 설정한다 
  wind_gust_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.X(), sqrt(wind_gust_direction_variance_)));
  wind_gust_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Y(), sqrt(wind_gust_direction_variance_)));
  wind_gust_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Z(), sqrt(wind_gust_direction_variance_)));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  // 업데이트 이벤트를 듣는다 
  // 이 이벤트를 매 시뮬레이션 iteration 마다 pub? 한다 
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWindPlugin::OnUpdate, this, _1));

  wind_pub_ = node_handle_->Advertise<physics_msgs::msgs::Wind>("~/" + wind_pub_topic_, 10);

#if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
#else
  last_time_ = world_->GetSimTime();
#endif

}

// This gets called by the world update start event.
// world update 시작 이벤트에 의해 호추 되는 함수 
// (이 플러그인은 world/model에 대한 의존성을 가지고 있음) 
void GazeboWindPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Get the current simulation time.
  // 현재 시뮬레이션시간을 가지고 온다 
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world_->SimTime();
#else
  common::Time now = world_->GetSimTime();
#endif
  if ((now - last_time_).Double() < pub_interval_ || pub_interval_ == 0.0) {
    return;
  }
  last_time_ = now;

  // Calculate the wind force.
  // Get normal distribution wind strength
  // 바람의 힘을 계산 
  // 일반적인 바람의 힘을 가지고 온다 
  double wind_strength = std::abs(wind_velocity_distribution_(wind_velocity_generator_));
  wind_strength = (wind_strength > wind_velocity_max_) ? wind_velocity_max_ : wind_strength;
  // Get normal distribution wind direction
  // 바람의 방향을 가지고 온다 
  ignition::math::Vector3d wind_direction;
  wind_direction.X() = wind_direction_distribution_X_(wind_direction_generator_);
  wind_direction.Y() = wind_direction_distribution_Y_(wind_direction_generator_);
  wind_direction.Z() = wind_direction_distribution_Z_(wind_direction_generator_);
  // Calculate total wind velocity
  // 총 바람의 속도를 계산
  ignition::math::Vector3d wind = wind_strength * wind_direction;

  ignition::math::Vector3d wind_gust(0, 0, 0);
  // Calculate the wind gust velocity.
  // 돌풍의 속도를 계산
  if (now >= wind_gust_start_ && now < wind_gust_end_) {
    // Get normal distribution wind gust strength
    // 일반적으로 분배 되는 돌풍의 힘을 가지고 온다 
    double wind_gust_strength = std::abs(wind_gust_velocity_distribution_(wind_gust_velocity_generator_));
    wind_gust_strength = (wind_gust_strength > wind_gust_velocity_max_) ? wind_gust_velocity_max_ : wind_gust_strength;
    // Get normal distribution wind gust direction
    // 일반적으로 분배 되는 돌풍의 방향을 가지고 온다
    ignition::math::Vector3d wind_gust_direction;
    wind_gust_direction.X() = wind_gust_direction_distribution_X_(wind_gust_direction_generator_);
    wind_gust_direction.Y() = wind_gust_direction_distribution_Y_(wind_gust_direction_generator_);
    wind_gust_direction.Z() = wind_gust_direction_distribution_Z_(wind_gust_direction_generator_);
    wind_gust = wind_gust_strength * wind_gust_direction;
  }

  
  gazebo::msgs::Vector3d* wind_v = new gazebo::msgs::Vector3d();
  wind_v->set_x(wind.X() + wind_gust.X());
  wind_v->set_y(wind.Y() + wind_gust.Y());
  wind_v->set_z(wind.Z() + wind_gust.Z());

  wind_msg.set_frame_id(frame_id_);
  wind_msg.set_time_usec(now.Double() * 1e6);
  wind_msg.set_allocated_velocity(wind_v);

  wind_pub_->Publish(wind_msg);
}

GZ_REGISTER_WORLD_PLUGIN(GazeboWindPlugin);
}
