#include "main.h"
pros::Motor FrontRightWheel(3, pros::E_MOTOR_GEARSET_06, false);
pros::Motor BackRightWheel(9, pros::E_MOTOR_GEARSET_06, false);
pros::Motor BackRightWheel2(7, pros::E_MOTOR_GEARSET_06, true);
pros::Motor FrontLeftWheel(1, pros::E_MOTOR_GEARSET_06, false);
pros::Motor BackLeftWheel(2, pros::E_MOTOR_GEARSET_06, false);
pros::Motor BackLeftWheel2(8, pros::E_MOTOR_GEARSET_06, true);

pros::Motor Intake(20, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor Puncher(13, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::ADIDigitalOut piston1(1);
// pros::Distance distance(16);

pros::Controller master(pros::E_CONTROLLER_MASTER);

// bool fast = true;
bool punching = false;
bool prev_punching = false;
bool isPistonUp = false;
bool intaking = false;
// bool detectable = true;
// bool detecting = true;
//PID for Intake
float kP = 0.5;
float kI = 0.2;
float kD = 0.3;
int targetValue = 600;

double TotalError = 0;
double Derivative = 0;
double error_old = 0;

std::string printed = "DETECTING!!";
void intake(){
    while (true) {
        // if (master.get_digital(pros:: E_CONTROLLER_DIGITAL_A)){
        //     fast = !fast;
        // }

        // if (fast){
        //     setpoint = Intake.get_voltage_limit();
        // } else {
        //     setpoint = Intake.get_voltage_limit() / 2;
        // }
        if (!intaking || !isPistonUp){
            Intake.move_velocity(0);
        } else{
            double rpm =  Intake.get_actual_velocity();

            double error_new = targetValue - rpm;
            TotalError += error_new;
            Derivative = error_new - error_old;
            double power = rpm + ((TotalError * kI) + (Derivative * kD) + (error_new * kP));

            Intake.move_velocity(power);
            error_old = error_new;

            pros::delay(20);

            // if (detectable && detecting && distance.get() < 100) {
            //     intaking = false;
            //     detectable = false;
            // }
            // if (!detectable && detecting && distance.get() >= 80) {detectable = true;}
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
            intaking = !intaking;
            if (intaking){
                targetValue = 600;
            }
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            targetValue = -targetValue;
            if (!intaking){
                intaking = true;
            }
        }
    }
}

void piston(bool up){
    piston1.set_value(up);
    isPistonUp = up;
}

void drive(int straight, int turn, bool brake = false) {
    // FrontRightWheel.brake();
    // BackRightWheel.brake();
    // BackRightWheel2.brake();
    // FrontLeftWheel.brake();
    // BackLeftWheel.brake();
    // BackLeftWheel2.brake();
    FrontRightWheel.move(straight - turn);
    BackRightWheel.move(straight - turn);
    BackRightWheel2.move(straight - turn);
    FrontLeftWheel.move(-straight - turn);
    BackLeftWheel.move(-straight - turn);
    BackLeftWheel2.move(-straight - turn);
}

void puncher(int vol){
    Puncher.move(vol);
}

void initialize(){
    piston(false);
}
void autonomous(){
    FrontRightWheel.move(-127);
    BackRightWheel.move(-127);
    FrontLeftWheel.move(127);
    BackLeftWheel.move(127);
    pros::delay(5000);
    drive(0,0);
    return;
}
void opcontrol() {
    new pros::Task(intake);
    master.set_text(0,0, printed);

    int straight;
    int turn;
    piston(false);
    FrontRightWheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    BackRightWheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    BackRightWheel2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    FrontLeftWheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    BackLeftWheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    BackLeftWheel2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    Puncher.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    while (true) {
        if (punching){
            puncher(127);
        } else {
            puncher(0);
        }
        straight = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        turn = master.get_analog(pros:: E_CONTROLLER_ANALOG_RIGHT_X);

        drive(straight, turn);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
            piston(!isPistonUp);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            punching = !punching;
        }

        // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
        //     detecting = !detecting;
        //     if (detecting){
        //         printed = "DETECTING!!";
        //     } else {
        //         printed = "NOT DETECTING!!";
        //     }
        //     master.set_text(0,0, printed);
    }
}