package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

        first_sample_slow();
        straightenup_robot();
        pick_sample(165,1200);
        haul_second();
        //pick_sample(190,1500);

        // LM3 CODE EXECUTES AS BELOW COMMENTED CODE.
        //first_sample();
        //second_sample();
        //third_sample();
        //sleep(500);
    }
    public void first_sample_slow() {
        // first routine
        DRIVE_SPEED_MULTIPLIER = 0.65;
        elbow_power = 1;
        moveElbow();
        sleep(100);
        //driveStraightPID(7);
        power_arm = 0.85; // was not here in working condition
        moveArm(); // was not here in working condition
        StrafingAUTO(12, false); // was 12 in working condition
        driveStraightPID_notimer(20.5);
        elbow_power = 4;
        moveElbow();
        turnPID_central(55, 20);
        drop_basket_slow();
    }
    public void drop_basket_slow() {
        // drop sample
        driveStraightPID_timer(200,0.5);
        power_arm = 0.2;
        moveArm();
        sleep(450); //2100 but we changed from 20:1 to 40:1
        power_arm = 0.05;//keep the arm in one place with almost no power
        moveArm();
        //DRIVE_SPEED_MULTIPLIER = 0.65;
        elbow_power = -0.5;
        moveElbow();
        sleep(400);
        OpenBaseClaw();
        sleep(400);

        elbow_power = 80;
        moveElbow();
        sleep(500);
        DRIVE_SPEED_MULTIPLIER = 0.4;
        driveStraightPID_timer(200,-0.5);
        DRIVE_SPEED_MULTIPLIER = 0.65;
        //sleep(100);
        power_arm = -0.95;//o used to be -10
        moveArm();
        sleep(1000);

        power_arm = 0;
        moveArm();
        sleep(100);


    }
    void straightenup_robot() {
        StrafingFAST(8,false);
        turnPID_central(215, 20);//205
        //turnPIDWTimer(-140,20);
        driveStraightPID_timer(700,-0.6);
        sleep(100);
        DRIVE_SPEED_MULTIPLIER = 0.65;
        driveStraightPID_notimer(16);
        while(distSensor.getDistance(DistanceUnit.CM) > 24) {
            driveStraightPID_timer(100,0.2);
        }
        driveStraightPID_timer(100,-0.5);

    }
    void test_Dist() {
        while(distSensor.getDistance(DistanceUnit.CM) > 24) {
            driveStraightPID_timer(100,0.2);
        }
    }
    void haul_second() {
        StrafingFAST(8,true);
        turnPID_central(220, 20);//225
        driveStraightPID_timer(800,-0.6);
        sleep(400);
        DRIVE_SPEED_MULTIPLIER = 0.95;
        driveStraightPID_notimer(52);
        StrafingFAST(10,true);
        driveStraightPID_notimer(-44);

        //while(distSensor.getDistance(DistanceUnit.CM) > 25) {
        //    driveStraightPID_timer(100,0.2);
       // }
    }

    void pick_sample(int angle_turn, int arm_sleep){
        elbow_power = -0.5;
        moveElbow();
        sleep(200); //1550 // Make 200 if does not work fast enough
        elbow_power = -0.1;
        moveElbow();
        sleep(800);
        //elbow_power = 0.01;
        //moveElbow();
        //sleep(1000);
        CloseBaseClaw();
        sleep(500);//used to be 1200
        power_arm = -0.7;
        moveArm();
        sleep(250);
        power_arm = 0.05;
        moveArm();
        elbow_power = 80;
        moveElbow();
        sleep(400);
        power_arm= 0.95;
        moveArm();
        DRIVE_SPEED_MULTIPLIER = 0.75;
        //start of delivering 2nd sample to the basket
        turnPID_central(angle_turn, 20);
        //driveStraightPID(7);
        sleep(arm_sleep);
        power_arm = 0.05;//keep the arm in one place with almost no power
        moveArm();
        driveStraightPID_timer(800,0.55); // perfectly working with 10.. moving to see third sample is possible
        elbow_power = -0.5;
        moveElbow();
        sleep(800);
        OpenBaseClaw();
        sleep(400);
        // end of 2nd routine
        elbow_power = 4;
        moveElbow();
        sleep(200);//1000
        DRIVE_SPEED_MULTIPLIER = 0.5;
        driveStraightPID_timer(800,-0.5); // move to -4 if remove below code
        sleep(100);
        DRIVE_SPEED_MULTIPLIER = 0.75;
        power_arm = -10;//also used to be -10
        moveArm();
        sleep(1200); // move to 200 if remove below code
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


