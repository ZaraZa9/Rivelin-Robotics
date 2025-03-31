#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

const double PI = 3.141592653589793;

struct JointState {
    double U1, U2, R1, R2, P1;
};

struct Position {
    double x, z, orient;
};

Position next_coordinates(double init_x, double init_y, double arm_length, double current_orientation, double new_angle) {
    Position pos;
    pos.x = init_x + arm_length * sin((new_angle + current_orientation) * PI / 180);
    pos.z = init_y + arm_length * cos((new_angle + current_orientation) * PI / 180);
    pos.orient = new_angle + current_orientation;
    return pos;
}

vector<JointState> generate_trajectory(JointState start, JointState target, double time_step) {
    
    double max_time = 0.0;
    vector<JointState> trajectory;
    
    vector<double> changes_in_angles = { target.U1 - start.U1, target.U2 - start.U2, target.R1 - start.R1, target.R2 - start.R2, target.P1 - start.P1 };
    vector<double> speeds = {30, 30, 45, 45, 10};
    vector<double> acceleration = {120, 120, 180, 180, 50};

    for (size_t i = 0; i < changes_in_angles.size(); i++) {
        double changes = abs(changes_in_angles[i]);
        double v_max = speeds[i];
        double a = acceleration[i];
        
        //check if the max speed is reaced
        double s_reach_vmax = (v_max * v_max) / (2 * a); //kinematics equation
        double time;
        
        //if reached not reached, do not count the time to reach max speed
        if (changes <= s_reach_vmax) {
            time = sqrt(2 * changes / a);
        } else {
        //if reached max speed, calculate the time to reach max speed and the time at constant speed
            double t_accel = v_max / a;
            double s_accel = s_reach_vmax;
            double s_remaining = changes - s_accel;
            double t_constant = s_remaining / v_max;
            time = t_accel + t_constant;
        }
        max_time = max(max_time, time);
        //cout << "time: " << time << "\n";
    }

    int steps = (int)(max_time / time_step);
    cout << "steps: " << steps << "\n";
    
    for (int i = 0; i <= steps; i++) {
        double delta = (double)i / steps;
        JointState state;
        state.U1 = start.U1 + delta * changes_in_angles[0];
        state.U2 = start.U2 + delta * changes_in_angles[1];
        state.R1 = start.R1 + delta * changes_in_angles[2];
        state.R2 = start.R2 + delta * changes_in_angles[3];
        state.P1 = start.P1 + delta * changes_in_angles[4];

        trajectory.push_back(state);
    }

    return trajectory;
}

int main() {

    JointState start;
    JointState target;

    cout << "Input Initial Joint State U1, U2, R1, R2, P1\n";
    cin >> start.U1 >> start.U2 >> start.R1 >> start.R2 >> start.P1;
    cout << "Input Target Joint State U1, U2, R1, R2, P1\n";
    cin >> target.U1 >> target.U2 >> target.R1 >> target.R2 >> target.P1;


    double base_link = 300.0;
    double proximal_link = 400.0;
    double distal_link = 350.0;
    double holder_link = 50.0;

    

    if (target.U1 < -90 || target.U1 > 90 || target.U2 < -90 || target.U2 > 90 || target.R1 < -150 || target.R1 > 150 || target.R2 < -150 || target.R2 > 150 || target.P1 < 0 || target.P1 > 50) {
        cout << "Unreachable\n";
    }

    Position pos1 = next_coordinates(0, base_link, proximal_link, 90.0, target.U2);
    if (pos1.z < 0) {
        cout << "Collision\n";
    }

    Position pos2 = next_coordinates(pos1.x, pos1.z, distal_link, pos1.orient, target.R1);
    if (pos2.z < 0) {
        cout << "Collision\n";
    }

    Position pos3 = next_coordinates(pos2.x, pos2.z, holder_link, pos2.orient, target.R2);
    if (pos3.z < 0) {
        cout << "Collision\n";
    }

    Position pos4 = next_coordinates(pos3.x, pos3.z, target.P1, pos3.orient, 0);
    if (pos4.z < 0) {
        cout << "Collision\n";
    }

    double x_coordinate = pos4.x * cos(target.U1*PI / 180);
    double y_coordinate = pos4.x * sin(target.U1*PI / 180);
    double z_coordinate = pos4.z;
    std::cout << "Final coordinates: (" << x_coordinate << ", " << y_coordinate << ", " << z_coordinate << ")\n";

    cout << "trajectory\n";
    vector<JointState> trajectory = generate_trajectory(start, target, 1.0 / 150.0);
    for (JointState state : trajectory) {
        cout << "(" << state.U1 << ", " << state.U2 << ", " << state.R1 << ", " << state.R2 << ", " << state.P1 << ")\n";
    }

    return 0;
}
