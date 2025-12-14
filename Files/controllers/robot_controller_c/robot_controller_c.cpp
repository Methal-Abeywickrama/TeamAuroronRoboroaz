#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>
#include <cctype>
#include <cmath>
#include <memory>
#include "nav.h"

using namespace webots;
using namespace std;

// --- Webots & Physics Constants ---
#define TIME_STEP 64
#define MAX_SPEED 6.28

// Real E-puck Dimensions (Meters)
#define REAL_WHEEL_RADIUS 0.0205 
#define REAL_AXLE_LENGTH  0.052  
#define CELL_SIZE 0.25           

// Calculate Errors based on user formulas
const double R_ERROR = 0.05; // Tuned for reliability
const double L_ERROR = 0.01; // Tuned for reliability

// --- DRIVER CLASS (Interface to Webots) ---
class EPuckDriver {
public:
    Robot *robot;
    Motor *leftMotor, *rightMotor;
    PositionSensor *leftPS, *rightPS;
    double oldL, oldR;

    EPuckDriver() {
        robot = new Robot();
        leftMotor = robot->getMotor("left wheel motor");
        rightMotor = robot->getMotor("right wheel motor");
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        leftMotor->setVelocity(0.0);
        rightMotor->setVelocity(0.0);

        leftPS = robot->getPositionSensor("left wheel sensor");
        rightPS = robot->getPositionSensor("right wheel sensor");
        leftPS->enable(TIME_STEP);
        rightPS->enable(TIME_STEP);
        oldL = 0.0;
        oldR = 0.0;
    }

    ~EPuckDriver() { delete robot; }

    // Critical: Returns false if Webots stops
    bool step(Odometry &odom) {
        if (robot->step(TIME_STEP) == -1) return false;

        double curL = leftPS->getValue();
        double curR = rightPS->getValue();
        
        // Calculate velocity in rad/s
        double vl = (curL - oldL) / (TIME_STEP / 1000.0);
        double vr = (curR - oldR) / (TIME_STEP / 1000.0);
        
        oldL = curL;
        oldR = curR;

        odom.update(vl, vr, TIME_STEP / 1000.0);
        return true;
    }

    void setSpeeds(double l, double r) {
        // Clamp
        if(l > MAX_SPEED) l = MAX_SPEED; if(l < -MAX_SPEED) l = -MAX_SPEED;
        if(r > MAX_SPEED) r = MAX_SPEED; if(r < -MAX_SPEED) r = -MAX_SPEED;
        leftMotor->setVelocity(l);
        rightMotor->setVelocity(r);
    }
};

// --- LOGIC FUNCTIONS (Implemented as requested) ---

double getSpeed() {
    return MAX_SPEED * 0.5; // Half speed for safety
}

double toAngle(int x, int y) {
    double angle;
    // Map grid direction to Angle (0=East/Right, PI/2=North/Up?) 
    // Logic from prompt:
    if (x == 0) {
        // if y=1 (North) -> 0. if y=-1 (South) -> PI
        angle = (y + 1) / 2 ? 0 : PI; 
    } else {
        // if x=1 (East) -> PI/2. if x=-1 (West) -> -PI/2
        angle = (x + 1) / 2 ? (PI / 2) : (-PI / 2);
    }
    return angle;
}

bool EREquivalence(double a, double b, double error) {
    // Normalize angles for comparison if they are angles
    // Simple bound check
    if ((a > (b - error)) && (a < (b + error))) {
        return true;
    }
    return false;
}

void Rotate(double angle, Odometry& odem, EPuckDriver& driver) {
    int leftCoeff = (angle >= 0) ? 1 : -1;
    int rightCoeff = -leftCoeff;
    
    // Target Absolute Angle
    double dest_Ang = odem.getAngle() + angle;
    
    // Normalize destination to -PI to PI for safety
    if(dest_Ang > PI) dest_Ang -= 2*PI;
    if(dest_Ang < -PI) dest_Ang += 2*PI;

    cout << "Rotating to: " << dest_Ang << endl;

    // Control Loop
    while (!EREquivalence(dest_Ang, odem.getAngle(), R_ERROR)) {
        
        double speed = getSpeed();
        
        // Simple P-Controller for smooth arrival
        double err = dest_Ang - odem.getAngle();
        // Handle wrap-around error
        if(err > PI) err -= 2*PI;
        if(err < -PI) err += 2*PI;

        // Slow down when close
        if(abs(err) < 0.2) speed *= 0.2;

        // Set left speed / right speed
        // If leftCoeff is 1 (Turn Left), Left Motor back, Right Motor fwd
        if(err > 0) { // Turn Left
            driver.setSpeeds(-speed, speed);
        } else { // Turn Right
            driver.setSpeeds(speed, -speed);
        }

        // CRITICAL: Update Simulation
        if (!driver.step(odem)) break;
    }
    
    driver.setSpeeds(0, 0);
}

void Movement(int x, int y, Odometry& odem, EPuckDriver& driver) {
    // Convert grid steps to World Meters
    double dist_x = x * odem.getCellSize();
    double dist_y = y * odem.getCellSize();

    double dest_x = odem.getX() + dist_x;
    double dest_y = odem.getY() + dist_y;
    
    double ang = toAngle(x, y); // Get target absolute angle
    
    cout << "Moving to: " << dest_x << ", " << dest_y << endl;

    // 1. Rotate to face target
    // Calculate difference between Target Angle and Current Angle
    double rot_needed = ang - odem.getAngle();
    
    // Normalize rotation
    while(rot_needed > PI) rot_needed -= 2*PI;
    while(rot_needed < -PI) rot_needed += 2*PI;

    if (!EREquivalence(ang, odem.getAngle(), R_ERROR)) {
        Rotate(rot_needed, odem, driver);
    }

    // 2. Drive to target
    while (!(EREquivalence(dest_x, odem.getX(), L_ERROR) && 
             EREquivalence(dest_y, odem.getY(), L_ERROR))) {
        
        // Simple corrections if angle drifts
        double current_err = ang - odem.getAngle();
        while(current_err > PI) current_err -= 2*PI;
        while(current_err < -PI) current_err += 2*PI;

        double correction = current_err * 2.0;
        double fwd_speed = getSpeed();

        driver.setSpeeds(fwd_speed - correction, fwd_speed + correction);
        cout<< odem.getX() << odem.getY();

        // CRITICAL: Update Simulation
        if (!driver.step(odem)) break;
    }
    
    // set motor speed 0
    driver.setSpeeds(0, 0);
}

// Changed return type to bool to signal when to quit
bool Move(Odometry& odem, EPuckDriver& driver) {
    // AUTOMATIC SEQUENCE METHOD
    static string sequence = "NNNNNWWNN"; 
    static int seqIndex = 0;

    if (seqIndex >= sequence.length()) {
        return false; // Stop when sequence finishes
    }

    char Direction = sequence[seqIndex++];
    cout << "Executing Sequence Step " << seqIndex << ": " << Direction << endl;

    switch (Direction) {
        case 'N': Movement(0, 1, odem, driver); break;
        case 'E': Movement(1, 0, odem, driver); break;
        case 'S': Movement(0, -1, odem, driver); break;
        case 'W': Movement(-1, 0, odem, driver); break;
        default: break;
    }
    return true; // Continue
}

// --- MAIN ---
int main(int argc, char **argv) {
    EPuckDriver driver;
    Odometry odem(REAL_WHEEL_RADIUS, REAL_AXLE_LENGTH, CELL_SIZE);
    
    // 10x10 Map centered at 0,0
    Map myMap(16, 16, {0,0}, CELL_SIZE); 

    cout << "--- Automatic Controller Started (Sequence: NSWWS) ---" << endl;

    // Initial Step to process sensors
    driver.step(odem);

    // Loop through the sequence
    while(true) {
        // If Move returns false (sequence done), break the loop
        if (!Move(odem, driver)) break;
        // Optional: Run one step after Move returns to ensure physics settle
        if (!driver.step(odem)) break;
    }

    // The loop is over.
    return 0;
}