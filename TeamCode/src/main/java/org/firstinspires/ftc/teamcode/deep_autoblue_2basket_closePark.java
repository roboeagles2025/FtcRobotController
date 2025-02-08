package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "AutoBasketNoPark", group = "Autonomous")
public  class deep_autoblue_2basket_closePark extends RoboEaglesAutoBase2425 {
    boolean close_farther = true;
    boolean no_park = false;
    boolean with_sample = true;
    @Override
    public void runOpMode() {
        battery_volt = hardwareMap.voltageSensor.iterator().next();

        MapDevicesTesting();
        newSensorTele();
        CloseBaseClaw();
        OpenBottomClaw();
        power_arm = -10;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep;
        power_arm = 0;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep;
        //elbow_power = 40;//lift up elbow to get ready for dropping
        //moveElbow();//lift up elbow to get ready for dropping
        // Enable the TFOD processor for our TeamProp Detection.
        while (!isStarted() && !isStopRequested()) {
            //   detectTeamProp();
            // Do all the other stuff
            newSensorTele();
            //telemetry.update();
            sleep(100);
        }

        first_sample();
        straightenup_robot();
        pick_sample(155,1300);
        straightenup_robot_second();
        pick_sample(190,1800);

        // LM3 CODE EXECUTES AS BELOW COMMENTED CODE.
        //first_sample();
        //second_sample();
        //third_sample();
        //sleep(500);
    }

    void straightenup_robot() {
        StrafingFAST(6,false);
        turnPID_central(205, 20);//225
        driveStraightPID_timer(500,-0.6);
        sleep(100);
        DRIVE_SPEED_MULTIPLIER = 0.65;
        driveStraightPID(11);
    }

    void straightenup_robot_second() {
        StrafingFAST(4,true);
        turnPID_central(220, 20);//225
        driveStraightPID_timer(600,-0.6);
        sleep(100);
        DRIVE_SPEED_MULTIPLIER = 0.85;
        driveStraightPID(12);
        StrafingFAST(7,true);
    }

    void pick_sample(int angle_turn, int arm_sleep){
        elbow_power = -0.5;
        moveElbow();
        sleep(200); //1550 // Make 200 if does not work fast enough
        elbow_power = 0.08;
        moveElbow();
        sleep(1400);
        CloseBaseClaw();
        sleep(500);//used to be 1200
        power_arm = -0.8;
        moveArm();
        sleep(250);
        power_arm = 0.05;
        moveArm();
        elbow_power = 80;
        moveElbow();
        sleep(800);


        //start of delivering 2nd sample to the basket
        turnPID_central(angle_turn, 20);
        //driveStraightPID(7);
        power_arm = 27;
        moveArm();
        sleep(arm_sleep);
        power_arm = 0.05;//keep the arm in one place with almost no power
        moveArm();
        driveStraightPID(8); // perfectly working with 10.. moving to see third sample is possible
        elbow_power = -5;
        moveElbow();
        sleep(200);
        OpenBaseClaw();
        sleep(200);
        // end of 2nd routine
        elbow_power = 4;
        moveElbow();
        sleep(200);//1000
        DRIVE_SPEED_MULTIPLIER = 0.5;
        driveStraightPID(-5); // move to -4 if remove below code
        sleep(100);
        DRIVE_SPEED_MULTIPLIER = 0.75;
        power_arm = -10;//also used to be -10
        moveArm();
        sleep(200); // move to 200 if remove below code
        power_arm = 0;
        moveArm();
        sleep(200);
    }

    void third_sample() {

        // TO BE REMOVED IF DOES NOT WORK 3rd sample
        turnPID_central(232,20);// was 25
        StrafingAUTO(7,false);
        //driveStraightPID(1.5);
        elbow_power = -0.45;
        moveElbow();
        sleep(200); //1550
        elbow_power = 0.05;
        moveElbow();
        sleep(1500);
        CloseBaseClaw();
        sleep(1000);
        power_arm = -0.5;
        moveArm();
        sleep(500);
        elbow_power = 40;
        moveElbow();
        sleep(200);
        //start of delivering 2nd sample to the basket
        StrafingAUTO(5,false);

        turnPID_central(180,20);
        //driveStraightPID(7);
        power_arm = 27;
        moveArm();
        sleep(2100);
        power_arm = 0.05;//keep the arm in one place with almost no power
        moveArm();
        driveStraightPID(5);
        elbow_power = -0.5;
        moveElbow();
        sleep(100);
        OpenBaseClaw();
        sleep(500);
        elbow_power = 4;
        moveElbow();
        sleep(500);//1000
        driveStraightPID(-4);
        sleep(100);
         }



    public void final_park(boolean close) {
        if (close) {
            turnPID(125, 25);
            driveStraightPID(80);
            turnPID(-90, 25);
            driveStraightPID(10);
        } else {
            turnPID(125, 25);
            driveStraightPID(90);
            turnPID(-90, 30);
            driveStraightPID(10);
        }
    }

}


