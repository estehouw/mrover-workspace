#include <cmath>
#include <iostream>


using namespace std;


void update_location(float &x_pos, float &y_pos, float &x_vel, float &y_vel, float x_acc, float y_acc, float &distance_traveled, float deg_yaw_per_second, float deg_pitch_per_second, float &bearing, float &pitch_angle, float time_delta){
    //Estimates distance using right-hand reiman sum
    bearing += deg_yaw_per_second * time_delta;
    pitch_angle += deg_pitch_per_second * time_delta;
    x_acc *= cos(pitch_angle) * sin(bearing);
    y_acc *= cos(pitch_angle) * cos(bearing);
    x_vel += x_acc * time_delta;
    y_vel += y_acc * time_delta;
    x_pos += x_vel * time_delta;
    y_pos += y_vel * time_delta;
}

void update_location2(float &x_pos, float &y_pos, float &x_vel, float &y_vel, float x_acc, float y_acc, float &distance_traveled, float deg_yaw_per_second, float deg_pitch_per_second, float &bearing, float &pitch_angle, float time_delta){
    float x_pos_left, x_pos_right, y_pos_left, y_pos_right, x_vel_left, x_vel_right, y_vel_left, y_vel_right;
    //Estimates distance using trapeziodal reiman sum

    //Left side using previous iteration's data
    x_pos_left = x_pos + x_vel * time_delta;
    y_pos_left = y_pos + y_vel * time_delta;
    x_vel_left = x_vel + x_acc * time_delta;
    y_vel_left = y_vel + y_acc * time_delta;

    //Right side using new data
    bearing += deg_yaw_per_second * time_delta;
    pitch_angle += deg_pitch_per_second * time_delta;
    x_acc *= cos(pitch_angle) * sin(bearing);
    y_acc *= cos(pitch_angle) * cos(bearing);
    x_vel_right = x_vel + x_acc * time_delta;
    y_vel_right = y_vel + y_acc * time_delta;
    x_pos_right = x_pos + x_vel * time_delta;
    y_pos_right = y_pos + y_vel * time_delta;

    //Final is an average of left and right sides
    x_pos = (x_pos_left + x_pos_right) / 2;
    y_pos = (y_pos_left + y_pos_right) / 2;
    x_vel = (x_vel_left + x_vel_right) / 2;
    y_vel = (y_vel_left + y_vel_right) / 2;

    //Distance from where the accelerometer started on flat plane
    distance_traveled = sqrt(pow(x_pos,2) + pow(y_pos,2));

}

int main(){
    cout << "Yo was gud" << endl;
}