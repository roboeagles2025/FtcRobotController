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

@Autonomous(name = "Blue25CloserBasketDouble", group = "Autonomous")
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

        autonomousStartBlueBasket(true);
        //turntest();
        //autonomousStartBlueBasketTemp();
        //telemetry.update();

        //waitForStart();
        //autonomousStartTest();
    }


    void autonomousStartBlueBasket(boolean blue_auto) {
        // first routine

        driveStraightPID(12); //move the robot 12 inches
        turnPID(90 * turn_value, 20);//turn the robot to the left also it used to be 10
        driveStraightPID(8);
        turnPID(45 * turn_value, 20);
        elbow_power = -4;//put the elbow up
        moveElbow();//add in elbow function
        sleep(500);//sleep
        power_arm = 15;//extend the arm up
        moveArm();//add in arm function
        sleep(2150);//sleep
        power_arm = 0.05;//keep the arm in one place with no power
        moveArm();//add in arm function
        sleep(500);//sleep
        driveStraightPID(15);//drive a feet
        sleep(500);//sleep
        brClaw.setPosition(0.3);//open right claw...also closing for this claw is 0.2
        blClaw.setPosition(0.6);//open left claw...also closing for this claw is 0.7
        sleep(1000);//sleep


        // second routine
        driveStraightPID(-14);//drive backwards a feet
        power_arm = -10;//detract the arm down
        moveArm();//add in arm function
        sleep(1300);//sleep
        power_arm = 0;//set to 0 power for no movements
        moveArm();//add in arm function
        sleep(500);

        turnPID(-145,20);
        sleep(500);
        driveStraightPID(10);
        power_arm = 3;//set to 0 power for no movements
        moveArm();//add in arm function
        sleep(650);

        power_arm = 0.05;//keep the arm in one place with no power
        moveArm();//add in arm function
        sleep(1000);//sleep
        brClaw.setPosition(0.7); // closed
        blClaw.setPosition(0.2);
        sleep(1600);
        power_arm = -10;//detract the arm down
        moveArm();//add in arm function
        sleep(1300);//sleep
        turnPID(165,20);
        driveStraightPID(10);
        elbow_power = -4;//put the elbow up
        moveElbow();//add in elbow function
        sleep(500);//sleep
        power_arm = 15;//extend the arm up
        moveArm();//add in arm function
        sleep(2150);//sleep
        power_arm = 0.05;//keep the arm in one place with no power
        moveArm();//add in arm function
        sleep(500);//sleep
        brClaw.setPosition(0.3);
        blClaw.setPosition(0.6);
        sleep(1000);
        driveStraightPID(-17);//drive backwards 17 inches
        power_arm = -10;//detract the arm down
        moveArm();//add in arm function
        sleep(1300);//sleep
    }

    public void turntest()  {
        driveStraightPID(4);
        turnPID(-42,20);
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


