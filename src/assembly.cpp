#include "vex.h"

using namespace vex;
bool goalBool = false;
bool colorOpen = true;
bool topDetected = false;
bool otherBall = false;
// bool is_red_alliance = mik::alliance_is_red;

// const int openAngle = 0; // degrees flap opens
// const int closeAngle = 90; // flap closed
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
    vex::optical topSensor

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
    topSensor(topSensor)

{};

// You want to call this function once in the user control function in main.
void Assembly::init() {//set velocites???????????????????????????????????????????????????????????????????????????????????????????????
    // Create the task to move the lift arm. We only want one task to be created
    park_change.close();// make it go up in beginning
    colorSensor.setLight(ledState::on);
    colorSensor.setLightPower(100, percent);
    topSensor.setLight(ledState::on);
    topSensor.setLightPower(100, percent);

    
    colorMotor.setPosition(0, degrees);

    lift_task = vex::task([]()
                          {
        
        // assembly.move_lift_arm();
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

    // if(Controller.ButtonY.pressing()){
    //     intake_motor_front.spin(fwd, 12, volt);

        
    //     // colorOpen = false;
    // }else if (Controller.ButtonA.pressing()){
    //     intake_motor_front.spin(fwd, -12, volt);
    // }
    // else
    // {
    //     colorOpen = true;
    // }
    topDetected = topSensor.isNearObject();

    if(Controller.ButtonR2.pressing()){
        if(!topDetected){
            intake_motor_front.spin(fwd, 12, volt);
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
        lift_arm_motors.spin(fwd, -2, volt);
        intake_motor_front.spin(fwd, -2, volt);
    }
    else if ((Controller.ButtonY.pressing()))
    {
                // colorMotor.spin(reverse, 12, volt);

        
    }
    else if ((Controller.ButtonA.pressing()))
    {
        // colorMotor.spin(fwd, 12, volt);
    }else{
        intake_motor_front.stop();
        // colorMotor.stop(brakeType::hold);
        lift_arm_motors.stop();
        
    }

    // if(btnX_new_press(Controller.ButtonX.pressing())){
    //     goalBool = !goalBool;
    // }
    // if (Controller.ButtonR2.pressing()) {
    //     // intake_motor_front.spin(fwd, 12, volt);
    //     if(goalBool){
    //         intake_motor_front.spin(fwd, 12, volt);
    //     }
    //     else{
    //         intake_motor_front.spin(fwd, 1, volt);
    //     }
    //     lift_arm_motors.spin(fwd, 12, volt);

    // } else if (Controller.ButtonR1.pressing()) {
    //     // intake_motor_front.spin(fwd, -12, volt);
    //     if(goalBool){
    //         intake_motor_front.spin(fwd, -12, volt);
    //     }else{
    //         intake_motor_front.spin(fwd, -1, volt);
    //     }
    //     lift_arm_motors.spin(fwd, -12, volt);
    // } else if (Controller.ButtonL2.pressing()) {
    //     lift_arm_motors.spin(fwd, 2, volt);
    //     intake_motor_front.spin(fwd, 2, volt);
    // } else if (Controller.ButtonL1.pressing()) {
    //     lift_arm_motors.spin(fwd, -2, volt);
    //     intake_motor_front.spin(fwd, -2, volt);
    // } 
    // else if (Controller.ButtonY.pressing()) {
    //     // colorMotor.spinToPosition(10, degrees);// kkkkk try this out
    //     //colorMotor.spinFor( .5, sec, 12, volt);


    //     colorMotor.spin(fwd);



    //     // colorMotor.spin(fwd,12, volt);
    //     wait(.5, sec);
    //     // colorMotor.spin(fwd, 12, volt);
    // } else if (Controller.ButtonA.pressing()) {

    //     colorMotor.spin(reverse);

    //     // colorMotor.spin(fwd,-12, volt);
    //     wait(.5, sec);
    //     // colorMotor.spinToPosition(-10, degrees);
    //     // colorMotor.spin(fwd, -12, volt);
    // } 
    // else {
    //     intake_motor_front.stop();
    //     // colorMotor.stop();
    //     lift_arm_motors.stop();
        

    // }
    
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
    if(btnLeft_new_press(Controller.ButtonLeft.pressing()) && btnRight_new_press(Controller.ButtonRight.pressing()) ){
        park_change.toggle();
    }
}
void Assembly::colorCode(){

    double hue = colorSensor.hue();
    
    bool objectDetected = colorSensor.isNearObject();
    if(objectDetected){
        // intake_motor_front.spin(reverse);

        if(hue >= 160 && hue <= 250){

        colorMotor.spin(fwd , 12, volt);
        wait(.2, seconds);
        colorMotor.spin(fwd, -12, volt);
        wait(.2, seconds);
        otherBall = false;
        }
        else if (hue >= 0 && hue <= 60 && otherBall == false)
        {
            colorMotor.spin(fwd, -12, volt);
            wait(.1, seconds);
            otherBall = true;
        }else if(otherBall){
            colorMotor.stop(brakeType::hold);
        }
    }
    
    else{
        colorMotor.stop(brakeType::hold);
        otherBall = false;
        // intake_motor_front.spin(fwd);

    }


    wait(.05, sec);
}