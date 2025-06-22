/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "pros/distance.hpp"
#include "EZ-Template/util.hpp"

namespace ez {
class mcl_distance_sensor{
 public:
  pros::Distance mcl_sensor;

  /**
   * Creates a new MCL distance sensor with a distance sensor.
   *
   * \param port
   *        1
   * \param x_offset
   *        assumed inches, this is the x offset according to your robot.
   * \param y_offset
   *        assumed inches, this is the y offset according to your robot.
   * \param direction
   *        The direction the distance sensor is facing according to the robot: frnt/bck/lft/rght.
   */
   mcl_distance_sensor(int port, double x_offset, double y_offset, e_mcl_distance_direction direction);

   double get_in();

   int get_mm();

   double x_offset_get();

   double y_offset_get();

   void x_offset_set();

   void y_offset_set();

   private:
   
};
}