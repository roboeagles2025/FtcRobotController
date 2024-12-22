package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "HangSpec2ClosePark", group = "Autonomous")
public class deep_autoblue_2hangspec_closePark extends RoboEaglesAutoBase2425 {

    boolean close_farther = true;
    boolean no_park = false;
    boolean with_sample = true;
    boolean close_simple = true;
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
        DRIVE_SPEED_MULTIPLIER = 0.675;
        elbow_power = 0.5;
        moveElbow();
        driveStraightPID(26);
        //sleep(1000);
        power_arm = 10;
        moveArm();
        sleep(700);//sleep used to be 550 but we changed from 20:1 to 40:1

        power_arm = 0;
        moveArm();

        elbow_power = -0.1;
        moveElbow();
        //This was 500ms, we changed it to give it time to retract
        // this is working fine keep 1s.
        sleep(500);//sleep

        /*
        power_arm = -0.5;
        moveArm();
        sleep(100);
*/
        //Changing the claw opening a bit more
        //This is because servo wires were swapped.
        brClaw.setPosition(0.45);
        blClaw.setPosition(0);
        //Original code
        //brClaw.setPosition(0.7);//open right claw...also closing for this claw is 0.2
        //blClaw.setPosition(0.2);//open left claw...also closing for this claw is 0.7
        sleep(1200);
        elbow_power = 0.8;
        moveElbow();
        sleep(500);
        DRIVE_SPEED_MULTIPLIER = 0.75;
        driveStraightPID(-11);
        sleep(500);
        power_arm = -10;
        moveArm();
        sleep(600);
        power_arm = 0;
        moveArm();
        StrafingAUTO(20,false);
        turnPID(-90,20);
        turnPID(-90,20);
        driveStraightPID(12);
        bottomrClaw.setPosition(0);
        bottomlClaw.setPosition(0.7);
        elbow_power = 0.8;
        moveElbow();
        sleep(1000);
    }
    public void final_park_hang(boolean close) {
        if (close) {
            driveStraightPID(13);
            StrafingAUTO(52,false);
            driveStraightPID(-13);
        } else {
            driveStraightPID(13);
            StrafingAUTO(65,false);
            driveStraightPID(-10);
        }
    }
}


