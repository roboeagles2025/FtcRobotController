package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "FBlue25HangSpec2SCloserPark", group = "Autonomous")
public class deep_autoblue_hangspec_with2_closePark extends RoboEaglesAutoBase2425 {

    boolean close_farther = true;
    boolean no_park = false;

    @Override
    public void runOpMode() {

        MapDevicesTesting();
        // Enable the TFOD processor for our TeamProp Detection.
        while (!isStarted() && !isStopRequested()) {
            //   detectTeamProp();
            // Do all the other stuff
            //telemetry.update();
            sleep(10);
        }
        //waitForStart();

        //autonomousStartBlueHangLower();
        autonomousStartBlueHangHigher();
        //telemetry.update();

        //waitForStart();
        //autonomousStartTest();
    }


    double distance;

    void autonomousStartBlueHangHigher() {

        elbow_power = -0.5;
        moveElbow();
        driveStraightPID(24);
        //sleep(1000);
        sleep(1000);
        power_arm = 10;
        moveArm();
        sleep(1000);//sleep used to be 550 but we changed from 20:1 to 40:1

        power_arm = 0;
        moveArm();


        elbow_power = 0.05;
        moveElbow();
        //This was 500ms, we changed it to give it time to retract
        // this is working fine keep 1s.
        sleep(1000);//sleep

        /*
        power_arm = -0.5;
        moveArm();
        sleep(100);
*/
        //Changing the claw opening a bit more
        //This is because servo wires were swapped.
        brClaw.setPosition(0);
        blClaw.setPosition(0.45);
        //Original code
        //brClaw.setPosition(0.7);//open right claw...also closing for this claw is 0.2
        //blClaw.setPosition(0.2);//open left claw...also closing for this claw is 0.7
        sleep(1200);
        elbow_power = -0.8;
        moveElbow();
        sleep(500);
        driveStraightPID(-12);
        sleep(500);
        power_arm = -10;
        moveArm();
        sleep(600);
        power_arm = 0;
        moveArm();
        //first strafe to go to sample after specimen
        StrafingAUTO(28,true);
        driveStraightPID(39);
        StrafingAUTO(9,true);
        driveStraightPID(-45);
        driveStraightPID(45);
        StrafingAUTO(10,true);
        driveStraightPID(-38);

        //parking time
        if(no_park == false) {
            final_park_hang(close_farther);
        }
        sleep(1500);
    }

    public void final_park_hang(boolean close) {
        if (close) {

            driveStraightPID(15);
            StrafingAUTO(56,false);
            driveStraightPID(-12);
        } else {
            driveStraightPID(14);
            StrafingAUTO(60,false);
            driveStraightPID(-12);
        }
    }
}


