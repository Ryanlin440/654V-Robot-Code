#pragma once

#include "vex.h"

using namespace vex;

enum lift_positions : int { IDLE = 229, LOADING = 206, SCORING = 99 };

class Assembly {
public:
    Assembly(
        mik::motor_group lift_arm_motors,
        mik::motor intake_motor_front, 
        vex::rotation lift_arm_encoder,
        mik::piston goal_change,
        mik::piston scraper_piston,
        mik::motor_group colorMotor,
        mik::piston park_change,
        vex::optical colorSensor,
        vex::optical topSensor


    );
    
    void init();
    void control();

    void move_lift_arm();
    void lift_arm_control();
    void intake_motors_control();
    void goal_change_control();
    void scraper_piston_control();
    void park_change_control();
    void colorCode();
    // void switchIntake();

    int lift_arm_position = IDLE;
    vex::task lift_task;
    
    mik::motor_group lift_arm_motors;
    mik::motor intake_motor_front;
    vex::rotation lift_arm_encoder;
    mik::piston goal_change;
    mik::piston scraper_piston;
    mik::motor_group colorMotor;
    mik::piston park_change;
    vex::optical colorSensor;
    vex::optical topSensor;

};