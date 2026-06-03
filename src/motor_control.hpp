#pragma once

// Point d'entrée unique : #include <motor_control.hpp> (Arduino / CMake).
#include "motor_control/current_loop_controller.hpp"
#include "motor_control/current_sensor.hpp"
#include "motor_control/dc_brushed_motor.hpp"
#include "motor_control/hg7881_bridge_pins.hpp"
#include "motor_control/hg7881_motor_driver.hpp"
#include "motor_control/motor_pwm_driver.hpp"
#include "motor_control/position_current_cascade.hpp"
#include "motor_control/position_sensor.hpp"
#include "motor_control/vnh7070as_bridge_pins.hpp"
#include "motor_control/vnh7070as_motor_driver.hpp"

// Utilisés par les .cpp de la bibliothèque ; inclus ici pour un seul #include dans chaque .cpp.
#include <algorithm>
#include <cmath>
