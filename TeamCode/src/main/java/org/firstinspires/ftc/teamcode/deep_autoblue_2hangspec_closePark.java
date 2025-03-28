package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "AutoHangSpec", group = "Autonomous")
public class deep_autoblue_2hangspec_closePark extends RoboEaglesAutoBase2425 {

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
        OpenBaseClaw();
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
            newSensorTele();

            telemetry.update();
            sleep(10);
        }
        //waitForStart();
        //autonomousStartBlueHangLower();
        elbow_power = 1;//lift up elbow to get ready for dropping
        moveElbow();//lift up elbow to get ready for dropping
        //autonomousStartBlueHangHigherNew();
        //Diagonal_autospec();
        //driveStraightPID(24);
        first_specimen_hang();
        second_specimen_hang();
        haul_samples();
        sleep(500);
        //telemetry.update();

        //waitForStart();
        //autonomousStartTest();
    }


    public void RobotOrientation(){
        //imu = hardwareMap.get(BNO055IMU.class,"imu");
        //resetIMU();
        double targetAngle = gyro.getAbsoluteHeading();
        telemetry.addData("Imu imu = %f", targetAngle);
        turnPID(targetAngle*-1,20);
    }
    public void first_specimen_hang() {

        //Start of first specimen pick up
        DRIVE_SPEED_MULTIPLIER = 0.5; //0.675 Setting up the first speed to be slow to decrease inefficiencies
        driveStraightPID_notimer(26.5);//go forward to the rung

        power_arm = 10;//lift up the arm to place specimen
        moveArm();//lift up the arm to place specimen
        sleep(700);//sleep
        power_arm = 0.01;//disengage arm to make the arm stay in place
        moveArm();//disengage arm to make the arm stay in place
        sleep(100);

        //Time based drive for 200ms to reach to the run without getting stuck at pid
        driveStraightPID_timer(200, 0.5);//go forward USED TO BE 18.5
        sleep(200);
        elbow_power = -0.1;
        moveElbow();
        sleep(100);
        power_arm = -10;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep;
        power_arm = 0;//disengage arm to make the arm stay in place
        moveArm();//disengage arm to make the arm stay in place
        sleep(200);
        OpenBaseClaw();
        sleep(500);//sleep
        //driveStraightPID(-1);//go forward to the rung
        elbow_power = 0.8;//lifts up elbow to grab the 2nd specimen
        moveElbow();//lifts up elbow to grab the 2nd specimen
        sleep(100);//sleep

    }

    public void second_specimen_hang() {


        driveStraightPID_notimer(-8);// go backward from submersible
        power_arm = -10;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep

        turnPID_central(95, 20);//turn
        //start of 2nd specimen pickup
        power_arm = 0;//keep the arm in place
        moveArm();//keep the arm in place
        DRIVE_SPEED_MULTIPLIER = 0.95;
        driveStraightPID_notimer(-34);
        turnPID_central(95, 20);//turn
        //drive 400ms full speed and 300ms half speed to reach to
        //wall and straighten the robot to avoid any angle issue.
        driveStraightPID_timer(400, 1);
        driveStraightPID_timer(300, 0.5);
        //Pickup second specimen
        CloseBottomClaw();
        sleep(500);//sleep
        power_arm = 10;// lift up arm to make sure the clip doesn't get stuck and break on the wall
        moveArm();// lift up arm to make sure the clip doesn't get stuck and break on the wall
        sleep(100);//sleep
        power_arm = 0.1;//keep the arm in place
        moveArm();//keep the arm in place
        sleep(150);

        elbow_power = 0.8;//move the elbow up so you can position properly on the rung
        moveElbow();//move the elbow up so you can position properly on the rung
        sleep(1000);//sleep
        driveStraightPID_notimer(-10);//go backward
        turnPID_central(90, 20);//turn // used to be 95
        DRIVE_SPEED_MULTIPLIER = 0.9;
        driveStraightPID_notimer(-40);//go forward
        turnPID_central(95, 20);//turn // used to be 9
        //going to the rungs
        DRIVE_SPEED_MULTIPLIER = 0.75;
        driveStraightPID_notimer(14);//go forward
        power_arm = 10;//bring the arm up
        moveArm();//bring the arm up
        sleep(1050);//sleep
        power_arm = 0.1;//keep the arm in place
        moveArm();//keep the arm in place
        sleep(200);
        //removing any pid stuck with free run based on time
        driveStraightPID_timer(200, 0.5);
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
        sleep(500);//sleep
        elbow_power = 40;
        moveElbow();
        sleep(500);//sleep;
        power_arm = 1;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep;
        power_arm = -0.1;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone

    }

    public void haul_samples() {

        //Sample hauling
        DRIVE_SPEED_MULTIPLIER  = 0.9;
        driveStraightPID_notimer(-10);//go forward to the rung
        //elbow_power = 0.8;//bring elbow up
        StrafingFAST(30,false);//strafe to park area
        turnPID_central(-10,0);
        driveStraightPID_notimer(32);//go forward to the rung
        //elbow_power = 0.8;//bring elbow up
        StrafingFAST(8,true);//strafe to park area
        driveStraightPID_notimer(-44);//go forward to the rung

        //Drive forward for second Haul
        driveStraightPID_notimer(44);//go forward to the rung
        //elbow_power = 0.8;//bring elbow up
        StrafingFAST(6,true);//strafe to park area
        driveStraightPID_notimer(-40);//go forward to the rung
        //End
    }
    public void Diagonal_autospec() {
        //Start of first specimen pick up
        DRIVE_SPEED_MULTIPLIER = 0.5; //0.675 Setting up the first speed to be slow to decrease inefficiencies
        driveStraightPID(26.5);//go forward to the rung
        power_arm = 10;//lift up the arm to place specimen
        moveArm();//lift up the arm to place specimen
        sleep(700);//sleep
        power_arm = 0.01;//disengage arm to make the arm stay in place
        moveArm();//disengage arm to make the arm stay in place
        sleep(100);
        driveStraightPID_timer(200,0.5);//go forward USED TO BE 18.5
        //driveStraightPID(1.2);//go forward to the rung
        sleep(200);
        elbow_power = -0.1;
        moveElbow();
        sleep(100);
        power_arm = -10;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep;
        power_arm = 0;//disengage arm to make the arm stay in place
        moveArm();//disengage arm to make the arm stay in place
        sleep(200);
        OpenBaseClaw();
        sleep(500);//sleep
        //driveStraightPID(-1);//go forward to the rung
        elbow_power = 0.8;//lifts up elbow to grab the 2nd specimen
        moveElbow();//lifts up elbow to grab the 2nd specimen
        sleep(100);//sleep
        driveStraightPID(-8);// go backward
        power_arm = -10;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep


        turnPID_central(95,20);//turn
        //start of 2nd specimen pickup
        power_arm = 0;//keep the arm in place
        moveArm();//keep the arm in place
        DRIVE_SPEED_MULTIPLIER = 0.95;
        driveStraightPID(-34);
        turnPID_central(95,20);//turn

        driveStraightPID_timer(400,1);
        driveStraightPID_timer(300,0.5);
        //driveStraightPID(-24);//go forward

        //turnPID_central(95,20);//turn
        //turnPID(95,20);//turn

        //driveStraightPID(-28);//go forward USED TO BE 18.5
        //StrafingAUTO(14, true); // was 12 in working condition
        //driveStraightPID_timer(1000,1);//go forward USED TO BE 18.5
        //driveStraightPID_timer(300,0.5);//go forward USED TO BE 18.5

        CloseBottomClaw();
        sleep(500);//sleep
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
        turnPID_central(90,20);//turn // used to be 95
        DRIVE_SPEED_MULTIPLIER = 0.9;
        driveStraightPID(-40);//go forward
        turnPID_central(95,20);//turn // used to be 9

        //going to the rungs
        DRIVE_SPEED_MULTIPLIER = 0.75;
        driveStraightPID(14);//go forward
        power_arm = 10;//bring the arm up
        moveArm();//bring the arm up
        sleep(1050);//sleep
        power_arm = 0.1;//keep the arm in place
        moveArm();//keep the arm in place
        sleep(200);
        driveStraightPID_timer(200,0.5);
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
        sleep(500);//sleep
        elbow_power = 40;
        moveElbow();
        sleep(500);//sleep;

        power_arm = 1;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep;
        power_arm = -0.1;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone

        //Sample hauling
        DRIVE_SPEED_MULTIPLIER  = 0.9;
        driveStraightPID(-10);//go forward to the rung
        //elbow_power = 0.8;//bring elbow up
        StrafingFAST(30,false);//strafe to park area
        driveStraightPID(32);//go forward to the rung
        //elbow_power = 0.8;//bring elbow up
        StrafingFAST(8,true);//strafe to park area
        driveStraightPID(-44);//go forward to the rung
        driveStraightPID(44);//go forward to the rung
        //elbow_power = 0.8;//bring elbow up
        StrafingFAST(6,true);//strafe to park area
        driveStraightPID(-40);//go forward to the rung

        //End
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


    public void driveStraightPID_notimer(double distance) {
        /*
        I don't think that we need the correction PID because as we are tracking the distance traveled by both the left set of wheels
        and the right set of wheels, if the robot starts to turn, the encoder values will differ and will automatically be corrected
         */
        lGroup.resetEncoder();
        rGroup.resetEncoder();
        gyro.reset();
        pidDriveLeft.reset();
        pidDriveRight.reset();
        int targetTicks = inchesToEncoderTicksInt(distance);
        pidDriveLeft.setSetPoint(targetTicks);
        pidDriveRight.setSetPoint(targetTicks);
        // I'm not sure whether FTCLib will be able to convert distance to encoder ticks,
        // If it doesn't work we can move over the encoderTicksToInches and inchesToEncoderTicks functions
        double speed = DRIVE_SPEED_MULTIPLIER; // Base speed that will be multiplied by the error
        // Previously used FTCLib's PositionControl, but that won't work as it isn't a PID Controller, only a PController,
        // so it gets stuck at lower speeds that the integral would have solved
        while (!pidDriveLeft.atSetPoint() && !pidDriveRight.atSetPoint() && opModeIsActive()) {
            double lPos = lGroup.getPositions().get(0);
            double rPos = rGroup.getPositions().get(0);
            double leftError = pidDriveLeft.calculate(lPos);
            double rightError = pidDriveRight.calculate(rPos);
            double leftSpeed = speed * leftError;
            double rightSpeed = speed * rightError;
            if (!pidDriveLeft.atSetPoint())
                if (leftSpeed > 0) {
                    leftSpeed = Range.clip(leftSpeed, 0.2, 1);
                    leftSpeed *= DRIVE_SPEED_MULTIPLIER;
                    leftSpeed = Range.clip(leftSpeed, 0.2, 1);
                }
                else {
                    leftSpeed = Range.clip(leftSpeed, -1, -0.2);
                    leftSpeed *= DRIVE_SPEED_MULTIPLIER;
                    leftSpeed = Range.clip(leftSpeed, -1, -0.2);
                }
            else
                leftSpeed = 0;

            if (!pidDriveRight.atSetPoint())
                if (rightSpeed > 0) {
                    rightSpeed = Range.clip(rightSpeed, 0.2, 1);
                    rightSpeed *= DRIVE_SPEED_MULTIPLIER;
                    rightSpeed = Range.clip(rightSpeed, 0.2, 1);
                }
                else {
                    rightSpeed = Range.clip(rightSpeed, -1, -0.2);
                    rightSpeed *= DRIVE_SPEED_MULTIPLIER;
                    rightSpeed = Range.clip(rightSpeed, -1, -0.2);
                }
            else
                rightSpeed = 0;

            lGroup.set(leftSpeed);
            rGroup.set(rightSpeed);
            telemetry.addData("Drive PID", "Target: %d, Traveled: %f, %f", targetTicks, lPos, rPos);
            telemetry.addData("Drive PID", "Left Speed: %f, Right Speed: %f", leftError, rightError);
            telemetry.addData("Distance in CM", "%.2f", distSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

        lGroup.stopMotor();
        rGroup.stopMotor();

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
