// Copyright 2024 Intelligent Robotics Lab
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

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "kobuki_ros_interfaces/msg/sound.hpp"
#include "move/controller.hpp"

using namespace std::chrono_literals;

namespace move
{

Controller::Controller(): Node("controller"), state_(FORWARD), bumper_pressed_(false)
{
  publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  sound_publisher_ = create_publisher<kobuki_ros_interfaces::msg::Sound>("/commands/sound", 10);
  bumper_subscriber_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
    "/events/bumper", 10, std::bind(&Controller::bumper_callback, this, std::placeholders::_1));
  start_time_ = this->now(); // Guardar tiempo de inicio
  timer_ = create_wall_timer(
    500ms, std::bind(&Controller::timer_callback, this));
}//Crea un temporizador que ejecuta timer_callback() cada 500ms

void
Controller::bumper_callback(const kobuki_ros_interfaces::msg::BumperEvent::SharedPtr msg)
{
  auto sound_message = kobuki_ros_interfaces::msg::Sound();
  if (msg->state == kobuki_ros_interfaces::msg::BumperEvent::PRESSED) {
    sound_message.value = kobuki_ros_interfaces::msg::Sound::ERROR; // Sonido cuando se presiona
    RCLCPP_WARN(this->get_logger(), "Bumper presionado, cambiando a modo RETROCESO.");
    bumper_pressed_ = true;
    state_ = BACKWARD;
    start_time_ = this->now();
  };
  sound_publisher_->publish(sound_message);
}

void
Controller::timer_callback()
{
  auto message = geometry_msgs::msg::Twist();
  rclcpp::Duration elapsed_time = this->now() - start_time_;
  //Obtiene el tiempo actual y calcula cuántos segundos han pasado desde que el nodo inició.
  
  
  switch (state_) {
    case FORWARD:  // SE puede añadir sentido de giro cuando sepamos diferenciar los bumpers
      message.linear.x = 0.2; //Avanza
      message.angular.z = 0.0;
      break;
      
    case BACKWARD:
      if (elapsed_time.seconds() < 5.0) {
        message.linear.x = -0.1;  // Velocidad en m/s
        message.angular.z = 0.0; // Sin giro
        } else {
          state_ = TURN;
          start_time_ = this->now();
        }
        break;
        
    case TURN:
      if (elapsed_time.seconds() < 2.0) {
        message.linear.x = 0.0;  // Velocidad en m/s
        message.angular.z = 1.57; // Con giro
      } else {
        state_ = FORWARD;
        bumper_pressed_ = false;
        start_time_ = this->now();
      }
      break;
      
    default:
      message.linear.x = 0.2;
      message.angular.z = 0.0;
      break;
      
    
  }

  publisher_->publish(message);//Envía el mensaje de velocidad a /cmd_vel.
  RCLCPP_INFO(this->get_logger(), "Publicando velocidad: x=%.2f, z=%.2f", message.linear.x, message.angular.z);
}

}  // namespace move

