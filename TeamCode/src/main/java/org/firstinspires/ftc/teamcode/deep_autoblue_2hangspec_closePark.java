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
        sleep(500);
        //telemetry.update();

        //waitForStart();
        //autonomousStartTest();
    }



    public void turnPID(int angle, int tolerance) {
        double speed_multiplier = 0.58;
        if (angle<0) {
            speed_multiplier = -0.58;
        }
        flDriveEx.set(-speed_multiplier);
        frDriveEx.set(speed_multiplier*1.5);
        blDriveEx.set(-speed_multiplier);
        brDriveEx.set(speed_multiplier*1.5);
        sleep(Math.abs(angle)*6);
        flDriveEx.set(0);
        frDriveEx.set(0);
        blDriveEx.set(0);
        brDriveEx.set(0);
        sleep(500);
    }
    void autonomousStartBlueHangHigher() {
        //Start of first specimen pick up
        DRIVE_SPEED_MULTIPLIER = 0.5; //0.675;
        brClaw.setPosition(0);
        blClaw.setPosition(0.45);
        sleep(1000);
        elbow_power = 0.5;
        moveElbow();
        driveStraightPID(26);
        power_arm = 10;
        moveArm();
        sleep(550);
        power_arm = 0;
        moveArm();
        elbow_power = -0.1;
        moveElbow();
        sleep(600);
        brClaw.setPosition(0.45);
        blClaw.setPosition(0);
        sleep(1200);
        elbow_power = 0.8;
        moveElbow();
        sleep(500);
        //DRIVE_SPEED_MULTIPLIER = 0.75;


        //start of 2nd specimen pickup
        driveStraightPID(-11);
        sleep(500);
        power_arm = -10;
        moveArm();
        sleep(600);
        power_arm = 0;
        moveArm();
        turnPID(-90,20);
        driveStraightPID(32);
        turnPID(-90,20);
        driveStraightPID(14);

        bottomrClaw.setPosition(0);
        bottomlClaw.setPosition(0.7);
        sleep(1000);
        power_arm = 10;
        moveArm();
        sleep(100);
        power_arm = 0;
        moveArm();

        elbow_power = 0.8;
        moveElbow();
        sleep(1000);
        driveStraightPID(-12);
        turnPID(-90,20);
        driveStraightPID(38);
        turnPID(-90,20);
        driveStraightPID(12);
        power_arm = 10;
        moveArm();
        sleep(1000);
        power_arm = 0;
        moveArm();
        elbow_power = -0.1;
        moveElbow();
        sleep(550);
        bottomrClaw.setPosition(0.7);
        bottomlClaw.setPosition(0);
        sleep(1000);
        elbow_power = 0.8;
        moveElbow();
        sleep(500);
        //Start of Park
        driveStraightPID(-11);
        sleep(500);
        power_arm = -10;
        moveArm();
        sleep(600);
        power_arm = 0;
        moveArm();
        driveStraightPID(-8);
        StrafingAUTO(20,false);
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


