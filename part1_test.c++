#include <iostream>
#include <cmath>
#include <vector>

const double PI = 3.141592653589793;

struct JointState {
    double U1, U2, R1, R2, P1;
};

struct Position {
    double x, z, orient;
};

struct coordinates {
    double x, y, z;
};

Position next_coordinates(double init_x, double init_y, double arm_length, double current_orientation, double new_angle) {
    Position pos;
    pos.x = init_x + arm_length * sin((new_angle + current_orientation)*PI / 180);
    pos.z = init_y + arm_length * cos((new_angle + current_orientation)*PI / 180);
    pos.orient = new_angle + current_orientation;
    return pos;
}

int main() {
    std::vector<JointState> testCases = {
        {0, 0, 0, 0, 0},
        {45, -90, -90, -90, 60},
        {-45, 90, -135, 45, 0},
        {90, 45, -30, 0, 50},
        {-45, 40, 140, 0, 50}
    };

    double base_link = 300.0;
    double proximal_link = 400.0;
    double distal_link = 350.0;
    double holder_link = 50.0;


    for (JointState testCase : testCases) {
        double U1 = testCase.U1;
        double U2 = testCase.U2;
        double R1 = testCase.R1;
        double R2 = testCase.R2;
        double P1 = testCase.P1;

        if (U1 < -90 || U1 > 90 || U2 < -90 || U2 > 90 || R1 < -150 || R1 > 150 || R2 < -150 || R2 > 150 || P1 < 0 || P1 > 50) {
            std::cout << "Unreachable\n";
            continue;
        }



        
        Position pos1 = next_coordinates(0, base_link, proximal_link, 90.0, U2);
        if (pos1.z < 0){
            std::cout << "Collision\n";
            continue;
        }

        //std::cout << "Current U2 angle: " << U2 << "\n";
        //std::cout << "400.0 * sin(U2 + 0.0): " << 400.0 * sin(U2 * PI / 180) << "\n";
        //std::cout << "Position 1: (" << pos1.x << ", " << pos1.z << ")\n";
        Position pos2 = next_coordinates(pos1.x, pos1.z, distal_link, pos1.orient, R1);

        if (pos2.z < 0){
            std::cout << "Collision\n";
            continue;
        }

        //std::cout << "Position 2: (" << pos2.x << ", " << pos2.z << ")\n";
        Position pos3 = next_coordinates(pos2.x, pos2.z, holder_link, pos2.orient, R2);
        if (pos3.z < 0){
            std::cout << "Collision\n";
            continue;
        }

        //std::cout << "Position 3: (" << pos3.x << ", " << pos3.z << ")\n";
        Position pos4 = next_coordinates(pos3.x, pos3.z, P1, pos3.orient, 0);
        if (pos4.z < 0){
            std::cout << "Collision\n";
            continue;
        }

        //std::cout << "Position 4: (" << pos4.x << ", " << pos4.z << ")\n";

        double x_coordinate = pos4.x * cos(U1*PI / 180);
        double y_coordinate = pos4.x * sin(U1*PI / 180);
        double z_coordinate = pos4.z;
        std::cout << "Final coordinates: (" << x_coordinate << ", " << y_coordinate << ", " << z_coordinate << ")\n";

    }
    return 0;
}