package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "TurningImprovements", group = "Autonomous")
public class Turning_Improvements extends RoboEaglesAutoBase2425 {

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
        //autonomousStartBlueHangHigher();
        //telemetry.update();
        practice_turning();
        sleep(500);
        //waitForStart();
        //autonomousStartTest();
    }
    double distance;
    public void turnPID(int angle, int tolerance) {
        double speed_multiplier = 0.53;
        if (angle<0) {
            speed_multiplier = -0.53;
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
    void practice_turning() {
        turnPID(90,20);
    }



    void autonomousStartBlueHangHigher() {
        DRIVE_SPEED_MULTIPLIER = 0.675;
        elbow_power = -0.5;
        moveElbow();
        driveStraightPID(26);
        //sleep(1000);
        sleep(1000);
        power_arm = 10;
        moveArm();
        sleep(550);//sleep used to be 550 but we changed from 20:1 to 40:1

        power_arm = 0;
        moveArm();


        elbow_power = 0.1;
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
        DRIVE_SPEED_MULTIPLIER = 0.75;
        driveStraightPID(-11);
        sleep(500);
        power_arm = -10;
        moveArm();
        sleep(600);
        power_arm = 0;
        moveArm();
     if(with_sample) {
         //first strafe to go to sample after specimen
         StrafingAUTO(27, true);
         driveStraightPID(39);
         StrafingAUTO(9, true);
         driveStraightPID(-47);
         driveStraightPID(47);
         StrafingAUTO(11, true);
         driveStraightPID(-40);
         if(no_park == false) {
             final_park_hang(close_farther);
         }
         sleep(1500);
     }
     else {
         if (close_simple) {
          StrafingAUTO(16, false);
          driveStraightPID(-7);
         }
         else {
             StrafingAUTO(32, false);
             driveStraightPID(-7);
         }
             }


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


