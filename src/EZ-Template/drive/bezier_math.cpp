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

double Drive::get_bezier_legnth(bezier A){
    double l = 0;
    for(double i = 0; i < 999; i++){
        l = l + util::distance_to_point(bezier_sample(A, i / 1000), bezier_sample(A, (i + 1) / 1000));
    }
    return l;
}

void Drive::get_bezier_pose_list(std::vector<bezier> A, drive_directions dir){
    bezier_poses.clear();
    for (int i = 0; i < A.size(); i++){
        double legnth = get_bezier_legnth(A.at(i));
        for (double j = 0; j < legnth; j++){
            bezier_poses.push_back({bezier_sample(A.at(i), util::clamp((j + 1) / legnth, i + 1, i)),dir , A.at(i).speed});
        }
    }
    final_index = bezier_poses.size();
    double step = A.at(A.size() - 1).D.x - A.at(A.size() - 1).C.x;
    double slope = (A.at(A.size() - 1).D.y - A.at(A.size() - 1).C.y) / (A.at(A.size() - 1).D.x - A.at(A.size() - 1).C.x);
    pose final_pose = bezier_sample(A.at(A.size() - 1), 1);
    for(int i = 0; i < odom_look_ahead_get(); i++){
        bezier_poses.push_back({{final_pose.x + (i + 1) * A.at(A.size() - 1).C.x, slope * (final_pose.x + (i + 1) * A.at(A.size() - 1).C.x) + final_pose.y}, dir, A.at(A.size() - 1).speed});
    }
}