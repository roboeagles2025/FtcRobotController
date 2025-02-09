package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "AltHangSpec2ClosePark", group = "Autonomous")
public class alt_deep_autoblue_2hangspec_closePark extends RoboEaglesAutoBase2425 {


    boolean close_farther = true;
    boolean no_park = false;
    boolean with_sample = true;
    boolean close_simple = true;
    double speed_multiplier = 0.58;

    @Override
    public void runOpMode() {

        MapDevicesTesting();
        // RobotOrientation();
        //elbow_power = 1;//lift up elbow to get ready for dropping
        //moveElbow();//lift up elbow to get ready for dropping\
        power_arm = -10;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep;
        power_arm = 0;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep;
        CloseBaseClaw();
        OpenBottomClaw();
        sleep(500);
        battery_volt = hardwareMap.voltageSensor.iterator().next();
        battery_power = battery_volt.getVoltage();
        telemetry.addData("Battery voltage = %f", battery_power);
        telemetry.addData("motor power = %f", speed_multiplier);

        telemetry.update();

        // Enable the TFOD processor for our TeamProp Detection.
        while (!isStarted() && !isStopRequested()) {
            //   detectTeamProp();
            // Do all the other stuff
            //telemetry.update();
            battery_power = battery_volt.getVoltage();
            telemetry.addData("power = %f", battery_power);
            telemetry.addData("motor power = %f", speed_multiplier);

            telemetry.update();
            sleep(10);
        }
        //waitForStart();

        //autonomousStartBlueHangLower();
        elbow_power = 1;//lift up elbow to get ready for dropping
        moveElbow();//lift up elbow to get ready for dropping
        autonomousStartBlueHangHigherNew();
        //driveStraightPID(24);
        sleep(500);
        //telemetry.update();

        //waitForStart();
        //autonomousStartTest();
    }


    public void RobotOrientation() {
        //imu = hardwareMap.get(BNO055IMU.class,"imu");
        //resetIMU();
        double targetAngle = gyro.getAbsoluteHeading();
        telemetry.addData("Imu imu = %f", targetAngle);
        turnPID(targetAngle * -1, 20);
    }

    void autonomousStartBlueHangHigherNew() {
        //Start of first specimen pick up
        DRIVE_SPEED_MULTIPLIER = 0.5; //0.675 Setting up the first speed to be slow to decrease inefficiencies
        driveStraightPID(27.1);//go forward to the rung
        power_arm = 10;//lift up the arm to place specimen
        moveArm();//lift up the arm to place specimen
        sleep(650);//sleep
        power_arm = 0.01;//disengage arm to make the arm stay in place
        moveArm();//disengage arm to make the arm stay in place
        sleep(500);
        driveStraightPID(2.1);//go forward to the rung
        sleep(500);
        power_arm = -10;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(150);//sleep;
        power_arm = 0;//disengage arm to make the arm stay in place
        moveArm();//disengage arm to make the arm stay in place
        sleep(500);
        OpenBaseClaw();
        sleep(1200);//sleep
        //driveStraightPID(-1);//go forward to the rung
        elbow_power = 0.8;//lifts up elbow to grab the 2nd specimen
        moveElbow();//lifts up elbow to grab the 2nd specimen
        sleep(100);//sleep
        driveStraightPID(-12);// go backward
        sleep(500);//sleep
        power_arm = -10;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(300);//sleep;

        //start of 2nd specimen pickup
        power_arm = 0;//keep the arm in place
        moveArm();//keep the arm in place
        turnPID_central(85, 20);//turn
        DRIVE_SPEED_MULTIPLIER = 0.65;
        driveStraightPID(-34);//go forward

        turnPID_central(90, 20);//turn
        //turnPID(95,20);//turn

        //driveStraightPID_timer(16);//go forward USED TO BE 18.5
        CloseBottomClaw();
        sleep(1000);//sleep
        power_arm = 10;// lift up arm to make sure the clip doesn't get stuck and break on the wall
        moveArm();// lift up arm to make sure the clip doesn't get stuck and break on the wall
        sleep(100);//sleep
        power_arm = 0.1;//keep the arm in place
        moveArm();//keep the arm in place
        sleep(150);

        elbow_power = 0.8;//move the elbow up so you can position properly on the rung
        moveElbow();//move the elbow up so you can position properly on the rung
        sleep(1000);//sleep
        driveStraightPID(-10);//go backward
        turnPID_central(90, 20);//turn // used to be 95
        driveStraightPID(-40);//go forward
        turnPID_central(95, 20);//turn // used to be 9

        //going to the rungs
        driveStraightPID(14.5);//go forward
        power_arm = 10;//bring the arm up
        moveArm();//bring the arm up
        sleep(1050);//sleep
        power_arm = 0.1;//keep the arm in place
        moveArm();//keep the arm in place
        sleep(500);
        driveStraightPID(2);
        sleep(500);
        elbow_power = -0.1;
        moveElbow();
        sleep(300);
        power_arm = -10;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(150);//sleep;
        power_arm = 0;//disengage arm to make the arm stay in place
        moveArm();//disengage arm to make the arm stay in place
        sleep(500);
        OpenBottomClaw();
        sleep(1000);//sleep
        power_arm = 1;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep;
        power_arm = 0;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        DRIVE_SPEED_MULTIPLIER = 0.9;
        driveStraightPID(-15);//go forward to the rung
        //elbow_power = 0.8;//bring elbow up
        StrafingFAST(35, false);//strafe to park area


        //End
    }
    public void StrafingFAST(long distance, boolean turn) {
        //  strafe_power = 1;
        if (turn == false) {
            strafe_power = -1*strafe_power;
        }
        long speed_multiplier = distance * power_factor;
        flDriveEx.set(-strafe_power*1.5);
        frDriveEx.set(strafe_power*1.5);
        blDriveEx.set(strafe_power*1.5);
        brDriveEx.set(-strafe_power*1.5);
        sleep(speed_multiplier);
        flDriveEx.set(0);
        frDriveEx.set(0);
        blDriveEx.set(0);
        brDriveEx.set(0);
        sleep(200);

    }

    public void final_park_hang(boolean close) {
        if (close) {
            driveStraightPID(13);
            StrafingAUTO(52, false);
            driveStraightPID(-13);
        } else {
            driveStraightPID(13);
            StrafingAUTO(65, false);
            driveStraightPID(-10);
        }
    }
}




