#include "vex.h"

using namespace vex;
bool topDetected = false;
bool flapOpen = false;

// Pass in the devices we want to use
Assembly::Assembly(
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

) :
    // Assign the ports to the devices
    lift_arm_motors(lift_arm_motors),
    intake_motor_front(intake_motor_front),
    lift_arm_encoder(lift_arm_encoder),
    goal_change(goal_change), // Make sure when using a 3 wire device that isnt mik::piston you convert the port. `to_triport(PORT_A)`. goal
    scraper_piston(scraper_piston),
    colorMotor(colorMotor),
    park_change(park_change), //park for now changing
    colorSensor(colorSensor),
    topSensor(topSensor),
    parkSensor(parkSensor)

{};

// You want to call this function once in the user control function in main.
void Assembly::init() {
    // Create the task to move the lift arm. We only want one task to be created
    park_change.open();// make it go up in beginning
    colorSensor.setLight(ledState::on);
    colorSensor.setLightPower(100, percent);
    topSensor.setLight(ledState::on);
    topSensor.setLightPower(100, percent);
    parkSensor.setLight(ledState::on);
    parkSensor.setLightPower(100, percent);

    assembly.scraper_piston.close();
    assembly.goal_change.open();
    
    colorSort = vex::task([] {
    while (true) {

        switch (assembly.sortState) {

        case open_flap:
            assembly.colorMotor.spin(fwd, 12, volt);
            wait(0.75, seconds);
            flapOpen = true;
            break;

        case close_flap:
            assembly.colorMotor.spin(fwd, -5, volt);
            wait(0.5, seconds);
            flapOpen = false;
            break;

        case slow_reverse:
            assembly.colorMotor.spin(fwd, -2, volt);
            wait(0.1, seconds);
            break;

        case idle:
        default:
            assembly.colorMotor.stop(brakeType::hold);
            break;
        }

        // Reset after handling an action
        assembly.sortState = idle;

        wait(20, msec);
    }
    return 0;
});



    lift_task = vex::task([]()
                          {
        assembly.colorMotor.spin(fwd, -6, volt);
        wait(1, seconds);
        assembly.colorMotor.stop(brakeType::hold);
        wait(.5, seconds);
        assembly.colorMotor.resetPosition();
        flapOpen = false;
        return 0; });
    // To stop the task do `assembly.lift_task.stop();`
} 

// You want to put this function inside the user control loop in main.
void Assembly::control() {
    lift_arm_control();
    intake_motors_control();
    goal_change_control();
    scraper_piston_control();
    park_change_control();
    colorCode();
}

void Assembly::move_lift_arm() {
    // Create a proportional controller. Increase the P just enough so there isn't much oscillation.
    PID lift_PID(.1, 0, 0);
    while (true) {
        // How far we need to move until desired angle
        float error = lift_arm_position - lift_arm_encoder.angle();
        // Converting error into motor voltage
        float output = lift_PID.compute(error);
        lift_arm_motors.spin(fwd, output, volt);
        vex::this_thread::sleep_for(20);
    }
}

void Assembly::lift_arm_control() {
    // new_press macro only allows input to go through when button is pressed. Resets after button is released.
    // if (btnX_new_press(Controller.ButtonX.pressing())) {
    //     // If Up arrow is pressed it will swap lift positions between scoring and loading
    //     if (lift_arm_position != SCORING) {
    //         lift_arm_position = SCORING; // Lift task will read this value
    //     } else {
    //         lift_arm_position = LOADING;
    //     }
    // } else if (btnUp_new_press(Controller.ButtonUp.pressing())) {
    //     // If Up arrow is pressed it will swap lift positions between loading and idle
    //     if (lift_arm_position != LOADING) {
    //         lift_arm_position = LOADING;
    //     } else {
    //         lift_arm_position = IDLE;
    //     }
    // }
}

// Spins intake forward if L1 is being held, reverse if L2 is being held; stops otherwise
void Assembly::intake_motors_control() {
    topDetected = topSensor.isNearObject();

    if(Controller.ButtonR2.pressing()){
        if(!topDetected){
            intake_motor_front.spin(fwd, 2, volt);
        }else{
            intake_motor_front.stop();
        }
        lift_arm_motors.spin(fwd, 12, volt);
    }
    else if (Controller.ButtonR1.pressing())
    {
        lift_arm_motors.spin(fwd, -12, volt);
    }
    else if (Controller.ButtonL2.pressing())
    {
        lift_arm_motors.spin(fwd, 12, volt);
        intake_motor_front.spin(fwd, 12, volt);
    }
    else if (Controller.ButtonL1.pressing())
    {
        lift_arm_motors.spin(fwd, -12, volt);
        intake_motor_front.spin(fwd, -12, volt);
    }
    else if (Controller.ButtonX.pressing())
    {
        lift_arm_motors.spin(fwd, -4, volt);
        intake_motor_front.spin(fwd, -4, volt);
    }else{
        intake_motor_front.stop();
        lift_arm_motors.stop();
        
    }
    
}


// Extends or retracts piston when button A is pressed, can only extend or retract again until button A is released and pressed again
void Assembly::goal_change_control() {//port c or goal
    if (btnUp_new_press(Controller.ButtonUp.pressing())) {
        goal_change.close();
    }
    if (btnDown_new_press(Controller.ButtonDown.pressing())) {
        goal_change.open();
    }

}

void Assembly::scraper_piston_control(){// port b
    if(btnB_new_press(Controller.ButtonB.pressing())){
        scraper_piston.toggle();

    }
}

void Assembly::park_change_control(){//port a park for now

    bool objectDetected = parkSensor.isNearObject();

    if(btnLeft_new_press(Controller.ButtonLeft.pressing())){    
        if(objectDetected){
            lift_arm_motors.spin(reverse, 4, volt);
            wait(.5, seconds);
            lift_arm_motors.stop(coast);
            park_change.close();
        }
        while(!objectDetected){
            lift_arm_motors.spin(reverse, 4, volt);
            objectDetected = parkSensor.isNearObject();
            if(objectDetected){
                wait(.1, seconds);
                lift_arm_motors.stop(coast);
                park_change.close();
            }
        }
        
        
    }
    // }
    if(btnRight_new_press(Controller.ButtonRight.pressing())){        
        park_change.toggle();
    }
    

}

void Assembly::set_team_color(team_color color){
    this->color = color;
}

void Assembly::colorCode() {
    double hue = colorSensor.hue();
    bool objectDetected = colorSensor.isNearObject();

    bool isRed  = (hue <= 35 || hue >= 350);
    bool isBlue = (hue >= 150 && hue <= 250);

    if (!objectDetected) {
        sortState = idle;
        return;
    }

    // ---------------- RED TEAM ----------------
    if (color == team_color::RED) {

        // Blue ball → open flap
        if (isBlue && !flapOpen) {
            sortState = open_flap;
        }
        // Red ball → close flap
        else if (isRed && flapOpen) {
            sortState = close_flap;
        }
        // Red ball, flap already closed → slow correction
        else if (isRed && !flapOpen) {
            sortState = slow_reverse;
        }
        else {
            sortState = idle;
        }
    }

    // ---------------- BLUE TEAM ----------------
    else if (color == team_color::BLUE) {

        // Red ball → open flap
        if (isRed && !flapOpen) {
            sortState = open_flap;
        }
        // Blue ball → close flap
        else if (isBlue && flapOpen) {
            sortState = close_flap;
        }
        // Blue ball, flap closed → slow correction
        else if (isBlue && !flapOpen) {
            sortState = slow_reverse;
        }
        else {
            sortState = idle;
        }
    }
}
