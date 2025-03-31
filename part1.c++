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
    double base_link = 300.0;
    double proximal_link = 400.0;
    double distal_link = 350.0;
    double holder_link = 50.0;

    JointState input;
    std::cout << "Input Joint State U1, U2, R1, R2, P1\n";
    std::cin >> input.U1 >> input.U2 >> input.R1 >> input.R2 >> input.P1;
    double U1 = input.U1;
    double U2 = input.U2;
    double R1 = input.R1;
    double R2 = input.R2;
    double P1 = input.P1;

    if (U1 < -90 || U1 > 90 || U2 < -90 || U2 > 90 || R1 < -150 || R1 > 150 || R2 < -150 || R2 > 150 || P1 < 0 || P1 > 50) {
        std::cout << "Unreachable\n";
    }



    
    Position pos1 = next_coordinates(0, base_link, proximal_link, 90.0, U2);
    if (pos1.z < 0){
        std::cout << "Collision\n";
    }

    //std::cout << "Current U2 angle: " << U2 << "\n";
    //std::cout << "400.0 * sin(U2 + 0.0): " << 400.0 * sin(U2 * PI / 180) << "\n";
    //std::cout << "Position 1: (" << pos1.x << ", " << pos1.z << ")\n";
    Position pos2 = next_coordinates(pos1.x, pos1.z, distal_link, pos1.orient, R1);

    if (pos2.z < 0){
        std::cout << "Collision\n";
    }

    //std::cout << "Position 2: (" << pos2.x << ", " << pos2.z << ")\n";
    Position pos3 = next_coordinates(pos2.x, pos2.z, holder_link, pos2.orient, R2);
    if (pos3.z < 0){
        std::cout << "Collision\n";
    }

    //std::cout << "Position 3: (" << pos3.x << ", " << pos3.z << ")\n";
    Position pos4 = next_coordinates(pos3.x, pos3.z, P1, pos3.orient, 0);
    if (pos4.z < 0){
        std::cout << "Collision\n";
    }

    //std::cout << "Position 4: (" << pos4.x << ", " << pos4.z << ")\n";

    double x_coordinate = pos4.x * cos(U1*PI / 180);
    double y_coordinate = pos4.x * sin(U1*PI / 180);
    double z_coordinate = pos4.z;
    std::cout << "Final coordinates: (" << x_coordinate << ", " << y_coordinate << ", " << z_coordinate << ")\n";

    return 0;
}