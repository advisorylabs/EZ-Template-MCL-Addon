/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/api.hpp"
#include "EZ-Template/util.hpp"
#include "drive.hpp"

pose Drive::bezier_sample(bezier input, double step){
    pose new_pose;
    new_pose.x = pow(1 - step, 3) * input.A.x + pow(1 - step, 2) * step * input.B.x + pow(step, 2) * (1 - step) * input.C.x + pow(step, 3) * input.D.x;
    new_pose.y = pow(1 - step, 3) * input.A.y + pow(1 - step, 2) * step * input.B.y + pow(step, 2) * (1 - step) * input.C.y + pow(step, 3) * input.D.y;
    return new_pose;
}

double Drive::get_angle(pose A, pose B){
    return atan2(B.x - A.x, B.y - A.y);
}