package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "XXBlue25CloserBasketDouble", group = "Autonomous")
public  class deep_auto_bluecloserdoublebasket extends RoboEaglesAutoBase2425 {
    boolean close_farther = true;
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

        //StrafingAUTO(24);
        autonomousStartBlueBasket(true);
        //turntest();
        //autonomousStartBlueBasketTemp();
        //telemetry.update();

        //waitForStart();
        //autonomousStartTest();
    }


    void autonomousStartBlueBasket(boolean blue_auto) {
        // first routine
        StrafingAUTO(11, false);
        driveStraightPID(18);
        turnPID(45,20);
        //driveStraightPID(5);
        drop_basket();
        driveStraightPID(-10);//drive backwards a feet

        // second routine

        power_arm = -10;//detract the arm down
        moveArm();//add in arm function
        sleep(1300);//sleep
        power_arm = 0;//set to 0 power for no movements
        moveArm();//add in arm function
        sleep(500);

        // turn and extend
        turnPID(-140,20);
        driveStraightPID(4);
        elbow_power = 0;
        moveElbow();
        power_arm = 3;
        moveArm();//add in arm function
        sleep(700);


        power_arm = 0.05;//keep the arm in one place with no power
        moveArm();//add in arm function
        sleep(500);//sleep
        brClaw.setPosition(0.7); // closed
        blClaw.setPosition(0.2);
        sleep(1000);
        power_arm = -10;//detract the arm down
        moveArm();//add in arm function
        sleep(1300);//sleep

        turnPID(150,20);
        driveStraightPID(6);
        drop_basket();

        sleep(1300);//sleep
    }
    // drop sample
    public void drop_basket() {
        // drop sample
        elbow_power = -4;//put the elbow up
        moveElbow();
        sleep(500);//sleep
        power_arm = 15;//extend the arm up
        moveArm();
        sleep(2000);//sleep
        power_arm = 0.05;//keep the arm in one place with almost no power
        moveArm();
        driveStraightPID(10);
        elbow_power = 0;//put the elbow up
        moveElbow();//add in elbow function
        sleep(500);//sleep
        brClaw.setPosition(0.3);//open right claw...also closing for this claw is 0.2
        blClaw.setPosition(0.6);//open left claw...also closing for this claw is 0.7
        sleep(1000);//sleep
        elbow_power = -4;//put the elbow up
        moveElbow();//add in elbow function
        sleep(500);//sleep

    }
    public void turntest()  {
        driveStraightPID(4);
        turnPID(-42,20);
        sleep(500);
    }

    long power_factor = 1000/25;
    public void StrafingAUTO(long distance, boolean turn) {
        double  strafe_power = 0.5;
        if (turn == false);
          strafe_power = -0.5;
        long speed_multiplier = distance * power_factor;
        flDriveEx.set(-strafe_power);
        frDriveEx.set(strafe_power);
        blDriveEx.set(strafe_power);
        brDriveEx.set(-strafe_power);
        sleep(speed_multiplier);
        flDriveEx.set(0);
        frDriveEx.set(0);
        blDriveEx.set(0);
        brDriveEx.set(0);
        sleep(500);

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


