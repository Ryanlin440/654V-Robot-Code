#include "vex.h"

using namespace vex;
using namespace mik;

 

void reset_drive_encoders() {
        chassis.left_drive.resetPosition();
        chassis.right_drive.resetPosition();
}
float clamp_voltage(float v, float min_v) {
    if (v > 0 && v < min_v) return min_v;
    if (v < 0 && -v < min_v) return -min_v;
    return v;
}
void drive_pid(float target_inches, float max_voltage, float intake_start_inches){
    reset_drive_encoders();

    const float wheel_diameter = 3.25;
    const float ratio = 1.0;
    const float circumference = wheel_diameter * 3.14159;

    float kP = .4;
    float kI = 0.0;
    float kD = 1.4;

    float error = 0;
    float last_error = 0;
    float integral = 0;
    float derivative = 0;

    float min_voltage = 2.5;

    // ---- ADDED VARIABLES ----
    float last_pos = 0;                  // where robot was last loop
    int stall_time = 0;                  // how long we have been stuck (ms)
    const int stall_threshold = 400;     // stop if stuck for 400ms (0.4 sec)

    bool intake_started = false;

    while (true)
    {
        float pos_deg =
            (chassis.left_drive.position(deg) + chassis.right_drive.position(deg)) / 2.0;

        float inches_moved = (pos_deg / 360.0) * (ratio * circumference);

        if (intake_start_inches != -1) {

    // If we haven't started it yet AND we passed the trigger distance
            if (!intake_started && inches_moved >= intake_start_inches) {

                assembly.lift_arm_motors.spin(fwd, 12, volt);
                assembly.intake_motor_front.spin(fwd, 12, volt);
                intake_started = true;    // So we don't restart it every loop
            }
        }

        error = target_inches - inches_moved;
        derivative = error - last_error;

        if (fabs(error) < 3) integral += error;

        float output = kP * error + kI * integral + kD * derivative;

        if (output > max_voltage) output = max_voltage;
        if (output < -max_voltage) output = -max_voltage;

        output = clamp_voltage(output, min_voltage);

        chassis.left_drive.spin(fwd, output, volt);
        chassis.right_drive.spin(fwd, output, volt);

        // ---- ADDED STALL DETECTION ----
        if (fabs(inches_moved - last_pos) < 0.02) {      // moved less than 0.02 inches?
            stall_time += 20;                            // count time not moving
        } else {
            stall_time = 0;                               // reset because we are moving again
        }

        last_pos = inches_moved;                          // update previous position

        if (stall_time > stall_threshold) {               // stuck for too long
            break;                                        // stop the PID loop
        }
        // ---- END OF ADDED CODE ----

        last_error = error;

        if (fabs(error) < 0.25) break;

        task::sleep(20);
    }

    chassis.left_drive.stop(hold);
    chassis.right_drive.stop(hold);
}


void default_constants(void) {
    chassis.set_control_constants(5, 10, 1.019, 5, 10, 1.019);

    // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
    chassis.set_turn_constants(10, .21, .01, 1.5, 15);//might need to increase this ask pranav aaaaaaaaaaaaaaaaaaaaaaaa
    chassis.set_drive_constants(10, 1.5, 0, 10, 0);

    chassis.set_heading_constants(6, .4, 0, 1, 0);
    chassis.set_swing_constants(12, .437, .0295, 3.486, 15);

    // Each exit condition set is in the form of (settle_error, settle_time, timeout).
    chassis.set_turn_exit_conditions(1.5, 75, 2000);
    chassis.set_drive_exit_conditions(1, 75, 3000);
    chassis.set_swing_exit_conditions(1.25, 75, 3000);
}

void odom_constants(void) {
    default_constants();
    chassis.heading_max_voltage = 10;
    chassis.drive_max_voltage = 8;
    chassis.drive_settle_error = 3;
    chassis.boomerang_lead = .5;
    chassis.boomerang_setback = 2;    
}



std::string template_auto(bool calibrate, auto_variation var, bool get_name) {
    /* The first variation will be this auto */
    if (var == one) {}

    /* We declare and allow a second variation of this auto; 
    You may want this if you want a different movements in the same starting configuration */
    if (var == two) { return template_auto_other_variation(calibrate, get_name); }

    if (get_name) { /* Give a desciption of your auto */ return "template auto 1 (3 objs)"; }
    if (calibrate) {
        /* Initialize robots starting position "https://path.jerryio.com/" and/or add extra movements to line up robots 
        starting position **IF MOVING DURING CALIBRATION DO BEFORE FIELD CONTROLLER PLUG IN** */
        chassis.set_coordinates(55, 23.5, 90);
    
        /* Example of turning before auto is ran */
        chassis.turn_max_voltage = 6; 
        chassis.turn_to_angle(45);

        return "";
    }
    
    /* We now run the auto */ 
    chassis.drive_distance(1);
    chassis.drive_distance(-1);

    return "";
}
std::string template_auto_other_variation(bool calibrate, bool get_name) {
    // Mirror template_auto() from the x-axis
    chassis.mirror_all_auton_y_pos();

    if (get_name) { return "template auto 2 (4 objs)"; }
    if (calibrate) {
        // Coordinates will be set to (55, -23.5) as y_pos is mirrored
        template_auto(calibrate, one, get_name);
        return "";
    }
    
    // Run auto, make sure to pass in one as var.
    template_auto(calibrate, one, get_name);

    return "";
}


std::string blue_left_winpoint(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "blue left winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }
    default_constants();
    turn_to_angle_params counterClockWise;
    turn_to_angle_params clockWise;
	counterClockWise.turn_direction = mik::direction::CCW;
	clockWise.turn_direction = mik::direction::CW;
    counterClockWise.timeout = 1000;
    clockWise.timeout = 1000;
    assembly.set_team_color(team_color::BLUE);
    drive_pid(26.5, 10, -1);
    chassis.turn_to_angle(345, counterClockWise);
    assembly.lift_arm_motors.spin(fwd, 12, volt);
    drive_pid(14, 4, -1);
    assembly.colorMotor.stop(brakeType::hold);
    chassis.turn_to_angle(45, clockWise);
    drive_pid(11.5, 8, 9);
    chassis.turn_to_angle(47);
    // wait(.5, seconds);

    assembly.lift_arm_motors.stop();
    assembly.intake_motor_front.stop();

    drive_pid(-43, 10, -1);
    
    chassis.turn_to_angle(180, clockWise);
    assembly.scraper_piston.toggle();
    wait(.2, seconds);
    assembly.lift_arm_motors.spin(fwd, 12 ,volt);
    drive_pid(15, 8, -1);
    
    wait(1, seconds);
    drive_pid(-10, 8, -1);
    
    assembly.scraper_piston.toggle();
    clockWise.timeout = 1500;
    assembly.goal_change.close();
    chassis.turn_to_angle(0);

    drive_pid(10, 8, 5);
    
    chassis.turn_to_angle(0);
    wait(2, seconds);
    return "";
}
std::string blue_left_sawp(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "blue left sawp"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }
    assembly.set_team_color(team_color::BLUE);
    return "";
}
std::string blue_left_elim(bool calibrate, auto_variation var, bool get_name) {   
    if (get_name) { return "blue left elim"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }
    drive_pid(5, 8, -1);
    assembly.set_team_color(team_color::BLUE);
    return "";
}
std::string blue_right_winpoint(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "blue right winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }
    default_constants();
    turn_to_angle_params counterClockWise;
    turn_to_angle_params clockWise;
	counterClockWise.turn_direction = mik::direction::CCW;
	clockWise.turn_direction = mik::direction::CW;
    counterClockWise.timeout = 1000;
    clockWise.timeout = 1000;
    assembly.set_team_color(team_color::BLUE);

  
    drive_pid(26.5, 10, -1);
    chassis.turn_to_angle(15, clockWise);
    assembly.lift_arm_motors.spin(fwd, 12, volt);
    drive_pid(14, 4, -1);
    assembly.colorMotor.stop(brakeType::hold);
    assembly.lift_arm_motors.stop(coast);
    chassis.turn_to_angle(315, counterClockWise);
    assembly.lift_arm_motors.spin(fwd, -12, volt);
    assembly.intake_motor_front.spin(fwd, -12, volt);
    drive_pid(11, 8, -1);
    chassis.turn_to_angle(315);
    wait(.5, seconds);

    assembly.lift_arm_motors.stop();
    assembly.intake_motor_front.stop();

    drive_pid(-48, 10, -1);
    
    chassis.turn_to_angle(180, counterClockWise);
    assembly.scraper_piston.toggle();
    wait(.5, seconds);
    assembly.lift_arm_motors.spin(fwd, 12 ,volt);
    drive_pid(15, 8, -1);
    
    wait(1, seconds);
    drive_pid(-5, 8, -1);
    assembly.scraper_piston.toggle();
    clockWise.timeout = 1500;
    chassis.turn_to_angle(10);
    assembly.goal_change.close();

    drive_pid(13, 8, 10);
    
    chassis.turn_to_angle(0);
    wait(2, seconds);
    return "";
}
std::string blue_right_sawp(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "blue right sawp"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}
std::string blue_right_elim(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "blue right elim"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    drive_pid(5, 8, -1);
    assembly.set_team_color(team_color::BLUE);
    return "";
}

std::string red_left_winpoint(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "red left winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);
        
        return "";
    }
    //start
    default_constants();
    turn_to_angle_params counterClockWise;
    turn_to_angle_params clockWise;
	counterClockWise.turn_direction = mik::direction::CCW;
	clockWise.turn_direction = mik::direction::CW;
    counterClockWise.timeout = 1000;
    clockWise.timeout = 1000;
    assembly.set_team_color(team_color::RED);

  
    drive_pid(26.5, 10, -1);
    chassis.turn_to_angle(345, counterClockWise);
    assembly.lift_arm_motors.spin(fwd, 12, volt);
    drive_pid(14, 4, -1);
    assembly.colorMotor.stop(brakeType::hold);
    chassis.turn_to_angle(45, clockWise);
    drive_pid(11.5, 8, 9);
    chassis.turn_to_angle(47);
    // wait(.5, seconds);

    assembly.lift_arm_motors.stop();
    assembly.intake_motor_front.stop();

    drive_pid(-43, 10, -1);
    
    chassis.turn_to_angle(180, clockWise);
    assembly.scraper_piston.toggle();
    wait(.2, seconds);
    assembly.lift_arm_motors.spin(fwd, 12 ,volt);
    drive_pid(15, 8, -1);
    
    wait(1, seconds);
    drive_pid(-10, 8, -1);
    
    assembly.scraper_piston.toggle();
    clockWise.timeout = 1500;
    assembly.goal_change.close();
    chassis.turn_to_angle(0);

    drive_pid(10, 8, 5);
    
    chassis.turn_to_angle(0);
    wait(2, seconds);
   

    return "";
}
std::string red_left_sawp(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "red left sawp"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }
    default_constants();
    turn_to_angle_params counterClockWise;
    turn_to_angle_params clockWise;
	counterClockWise.turn_direction = mik::direction::CCW;
	clockWise.turn_direction = mik::direction::CW;
    counterClockWise.timeout = 1000;
    clockWise.timeout = 1000;
    assembly.set_team_color(team_color::RED);

    drive_pid(32, 10, -1);
    chassis.turn_to_angle(270, counterClockWise);
    assembly.scraper_piston.toggle();
    drive_pid(10, 4, 5);
    wait(3, seconds);
    drive_pid(-5, 4, -1);
    assembly.scraper_piston.toggle();
    chassis.turn_to_angle(90, clockWise);
    assembly.goal_change.close();
    assembly.lift_arm_motors.spin(fwd, 12, volt);
    drive_pid(11, 4, 7);
    wait(2, seconds);
    assembly.lift_arm_motors.stop();
    assembly.intake_motor_front.stop();
    drive_pid(-11, 4, -1);
    chassis.turn_to_angle(135, clockWise);
    drive_pid(32.55, 8, 25);
    assembly.intake_motor_front.stop();
    // assembly.scraper_piston.toggle();
    assembly.goal_change.open();
    drive_pid(12.729, 8, 12);
    wait(1, seconds);
    assembly.lift_arm_motors.stop();
    assembly.intake_motor_front.stop();
    drive_pid(-12.729, 8, -1);
    chassis.turn_to_angle(180, clockWise);
    // assembly.lift_arm_motors.spin(fwd);
    drive_pid(46, 10, 40);
    assembly.intake_motor_front.stop();
    chassis.turn_to_angle(225, clockWise);
    drive_pid(32.55, 8, -1);
    chassis.turn_to_angle(270, clockWise);
    assembly.scraper_piston.toggle();
    assembly.lift_arm_motors.spin(fwd, 12, volt);
    drive_pid(5, 4, -1);
    wait(1, seconds);
    drive_pid(-5, 4, -1);
    assembly.scraper_piston.toggle();
    assembly.lift_arm_motors.stop();
    chassis.turn_to_angle(90, clockWise);
    assembly.goal_change.close();
    drive_pid(11, 4, 5);
    wait(2, seconds);

    
    return "";
}
std::string red_left_elim(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "red left elim"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);
        
        return "";
    }
    drive_pid(5, 8, -1);
    assembly.set_team_color(team_color::RED);
    return "";
}
std::string red_right_winpoint(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "red right winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);
        
        return "";
    }
     default_constants();
    turn_to_angle_params counterClockWise;
    turn_to_angle_params clockWise;
	counterClockWise.turn_direction = mik::direction::CCW;
	clockWise.turn_direction = mik::direction::CW;
    counterClockWise.timeout = 1000;
    clockWise.timeout = 1000;
    assembly.set_team_color(team_color::RED);

  
    drive_pid(26.5, 10, -1);
    chassis.turn_to_angle(15, clockWise);
    assembly.lift_arm_motors.spin(fwd, 12, volt);
    drive_pid(14, 4, -1);
    assembly.colorMotor.stop(brakeType::hold);
    assembly.lift_arm_motors.stop(coast);
    chassis.turn_to_angle(315, counterClockWise);
    assembly.lift_arm_motors.spin(fwd, -12, volt);
    assembly.intake_motor_front.spin(fwd, -12, volt);
    drive_pid(11, 8, -1);
    chassis.turn_to_angle(315);
    wait(.5, seconds);

    assembly.lift_arm_motors.stop();
    assembly.intake_motor_front.stop();

    drive_pid(-48, 10, -1);
    
    chassis.turn_to_angle(180, counterClockWise);
    assembly.scraper_piston.toggle();
    wait(.5, seconds);
    assembly.lift_arm_motors.spin(fwd, 12 ,volt);
    drive_pid(15, 8, -1);
    
    wait(1, seconds);
    drive_pid(-5, 8, -1);
    assembly.scraper_piston.toggle();
    clockWise.timeout = 1500;
    chassis.turn_to_angle(10);
    assembly.goal_change.close();

    drive_pid(13, 8, 10);
    
    chassis.turn_to_angle(0);
    wait(2, seconds);
    return "";
}
std::string red_right_sawp(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "red right sawp"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);
        
        return "";
    }

    return "";
}
std::string red_right_elim(bool calibrate, auto_variation var, bool get_name) {   
    if (get_name) { return "red right elim"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }
    drive_pid(5, 8, -1);
    assembly.set_team_color(team_color::RED);
    return "";
}

std::string skills(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "skills"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }
    default_constants();
    turn_to_angle_params counterClockWise;
    turn_to_angle_params clockWise;
	counterClockWise.turn_direction = mik::direction::CCW;
	clockWise.turn_direction = mik::direction::CW;
    counterClockWise.timeout = 1000;
    clockWise.timeout = 1000;

    drive_pid(32, 10, -1);
    chassis.turn_to_angle(270, counterClockWise);
    assembly.scraper_piston.toggle();
    assembly.goal_change.toggle();
    drive_pid(-10, 4, 5);
    drive_pid(20, 4, 5);
    wait(3, seconds);
    drive_pid(-5, 4, -1);
    assembly.scraper_piston.toggle();
    wait(1, seconds);
    chassis.turn_to_angle(90, clockWise);
    wait(1, seconds);
    drive_pid(13.4, 8, 10);
    wait(5, seconds);
    drive_pid(-5, 8, -1);
    chassis.turn_to_angle(45);
    drive_pid(-32, 8, -1);
    chassis.turn_to_angle(10);
    drive_pid(10, 8, -1);
    drive_pid(-50, 12, -1);
    drive_pid(5, 10, -1);
    drive_pid(-5, 10, -1);
    chassis.turn_to_angle(0);
    chassis.turn_to_angle(180);
    return "";
}

