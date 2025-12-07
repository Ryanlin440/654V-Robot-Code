#pragma once

#include "vex.h"

using namespace vex;

enum lift_positions : int { IDLE = 229, LOADING = 206, SCORING = 99 };

enum team_color : int{RED = 0, BLUE = 1};

enum SortState : int
{
    idle,
    open_flap,
    close_flap,
    slow_reverse
};
class Assembly
{
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
        vex::optical topSensor,
        vex::optical parkSensor


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
    void set_team_color(team_color color);

    int lift_arm_position = IDLE;
    vex::task lift_task;

    int flap_target = 0; 
    vex::task colorSort;
    void flap_pid_task();

    team_color color;
    SortState sortState = SortState::idle;

    mik::motor_group lift_arm_motors;
    mik::motor intake_motor_front;
    vex::rotation lift_arm_encoder;
    mik::piston goal_change;
    mik::piston scraper_piston;
    mik::motor_group colorMotor;
    mik::piston park_change;
    vex::optical colorSensor;
    vex::optical topSensor;
    vex::optical parkSensor;
};