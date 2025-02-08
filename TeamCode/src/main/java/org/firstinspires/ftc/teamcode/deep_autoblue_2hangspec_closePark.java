package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
        driveStraightPID(26.5);//go forward to the rung

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


        driveStraightPID(-8);// go backward from submersible
        power_arm = -10;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep

        turnPID_central(95, 20);//turn
        //start of 2nd specimen pickup
        power_arm = 0;//keep the arm in place
        moveArm();//keep the arm in place
        DRIVE_SPEED_MULTIPLIER = 0.95;
        driveStraightPID(-34);
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
        driveStraightPID(-10);//go backward
        turnPID_central(90, 20);//turn // used to be 95
        DRIVE_SPEED_MULTIPLIER = 0.9;
        driveStraightPID(-40);//go forward
        turnPID_central(95, 20);//turn // used to be 9
        //going to the rungs
        DRIVE_SPEED_MULTIPLIER = 0.75;
        driveStraightPID(14);//go forward
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
        driveStraightPID(-10);//go forward to the rung
        //elbow_power = 0.8;//bring elbow up
        StrafingFAST(30,false);//strafe to park area
        turnPID_central(-10,0);
        driveStraightPID(32);//go forward to the rung
        //elbow_power = 0.8;//bring elbow up
        StrafingFAST(8,true);//strafe to park area
        driveStraightPID(-44);//go forward to the rung

        //Drive forward for second Haul
        driveStraightPID(44);//go forward to the rung
        //elbow_power = 0.8;//bring elbow up
        StrafingFAST(6,true);//strafe to park area
        driveStraightPID(-40);//go forward to the rung
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
}


