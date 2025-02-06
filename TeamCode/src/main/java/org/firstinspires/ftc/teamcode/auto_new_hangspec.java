package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "AutoHangSpecFast", group = "Autonomous")
public class auto_new_hangspec extends RoboEaglesAutoBase2425 {

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


    public void RobotOrientation(){
        //imu = hardwareMap.get(BNO055IMU.class,"imu");
        //resetIMU();
        double targetAngle = gyro.getAbsoluteHeading();
        telemetry.addData("Imu imu = %f", targetAngle);
        turnPID(targetAngle*-1,20);
    }
    //5.25
    /*
    public void driveStraightPID(double distance) {
        double speed_multiplier = 0.8;
        int sleep_multipler = 23;
        if(distance < 0) {
            speed_multiplier = speed_multiplier * -1;
        }
        int sleep_time;
        int sleep_distance;
        sleep_distance = (int) distance;
        double power_distance = (distance * 100%100) / 100;

        speed_multiplier = speed_multiplier*(1+power_distance);
        sleep_time = sleep_multipler*sleep_distance;

        flDriveEx.set(speed_multiplier);
        frDriveEx.set(speed_multiplier);
        blDriveEx.set(speed_multiplier);
        brDriveEx.set(speed_multiplier);

        sleep(Math.abs(sleep_time));

        flDriveEx.set(0);
        frDriveEx.set(0);
        blDriveEx.set(0);
        brDriveEx.set(0);
        sleep(500);
    }

     */
    public void turnPID(int angle, int tolerance) {
        double angle_nonzer_subtract = 0;
        if (battery_power > 13.9) {
            speed_multiplier = 0.53;
        } else if (battery_power > 13.75){
            speed_multiplier = 0.55;
            angle_nonzer_subtract = 0.55;
        }else if(battery_power > 13.5) {
            speed_multiplier = 0.565;
            angle_nonzer_subtract = 0.565;
        }else if (battery_power > 13.25) {
            speed_multiplier = 0.57;
            angle_nonzer_subtract = 0.57;
        } else if(battery_power > 13) {
            speed_multiplier = 0.575;
            angle_nonzer_subtract = 0.575;
        } else if (battery_power > 12.75) {
            speed_multiplier = 0.58 ;
            angle_nonzer_subtract = 0.58;
        } else if (battery_power > 12.50) {
            speed_multiplier = 0.585;
            angle_nonzer_subtract = 0.585;
        } else if (battery_power > 12.25) {
            speed_multiplier = 0.59;
            angle_nonzer_subtract = 0.59;
        } else if (battery_power > 12) {
            speed_multiplier = 0.60;
            angle_nonzer_subtract = 0.60;
        } else  {
            speed_multiplier = 0.55;
        }
        if (angle<0) {
            speed_multiplier = -angle_nonzer_subtract;
        }

        telemetry.addData("motor power = %f", speed_multiplier);
        telemetry.update();

        flDriveEx.set(-speed_multiplier);
        frDriveEx.set(speed_multiplier*1.5);
        blDriveEx.set(-speed_multiplier);
        brDriveEx.set(speed_multiplier*1.5);
        sleep(Math.abs(angle)*6);
        flDriveEx.set(0);
        frDriveEx.set(0);
        blDriveEx.set(0);
        brDriveEx.set(0);
        sleep(100);
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
        driveStraightPID_timer(2,0.5);//go forward to the rung
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
        turnPID_central(85,20);//turn
        DRIVE_SPEED_MULTIPLIER = 0.65;
        driveStraightPID(-34);//go forward

        turnPID_central(90,20);//turn
        //turnPID(95,20);//turn

        driveStraightPID_timer(16,0.5);//go forward USED TO BE 18.5
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
        turnPID_central(90,20);//turn // used to be 95
        driveStraightPID(-40);//go forward
        turnPID_central(95,20);//turn // used to be 9

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
        DRIVE_SPEED_MULTIPLIER  = 0.9;
        driveStraightPID(-15);//go forward to the rung
        //elbow_power = 0.8;//bring elbow up
        StrafingFAST(35,false);//strafe to park area


        //End
    }
    void autonomousStartBlueHangHigher() {
        //Start of first specimen pick up
        DRIVE_SPEED_MULTIPLIER = 0.5; //0.675 Setting up the first speed to be slow to decrease inefficiencies
        elbow_power = 0.5;//lift up elbow to get ready for dropping
        moveElbow();//lift up elbow to get ready for dropping
        driveStraightPID(25);//go forward to the rung
        power_arm = 10;//lift up the arm to place specimen
        moveArm();//lift up the arm to place specimen
        sleep(650);//sleep
        power_arm = 0;//disengage arm to make the arm stay in place
        moveArm();//disengage arm to make the arm stay in place
        elbow_power = -0.05;//disengage elbow to make the arm tilt to place
        moveElbow();//disengage elbow to make the arm tilt to place
        sleep(200);//sleep
        OpenBaseClaw();
        sleep(1200);//sleep
        elbow_power = 0.8;//lifts up elbow to grab the 2nd specimen
        moveElbow();//lifts up elbow to grab the 2nd specimen
        sleep(500);//sleep


        //start of 2nd specimen pickup
        driveStraightPID(-11);// go backward
        sleep(500);//sleep
        power_arm = -10;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(600);//sleep
        power_arm = 0;//keep the arm in place
        moveArm();//keep the arm in place
        turnPID(-90,20);//turn
        driveStraightPID(34);//go forward
        turnPID(-90,20);//turn
        driveStraightPID(12.5);//go forward
        CloseBottomClaw();
        sleep(1000);//sleep
        power_arm = 10;// lift up arm to make sure the clip doesn't get stuck and break on the wall
        moveArm();// lift up arm to make sure the clip doesn't get stuck and break on the wall
        sleep(100);//sleep
        power_arm = 0;//keep the arm in place
        moveArm();//keep the arm in place
        elbow_power = 0.8;//move the elbow up so you can position properly on the rung
        moveElbow();//move the elbow up so you can position properly on the rung
        sleep(1000);//sleep
        driveStraightPID(-12);//go backward
        turnPID(-90,20);//turn
        driveStraightPID(40);//go forward
        turnPID(-90,20);//turn
        driveStraightPID(11);//go forward
        power_arm = 10;//bring the arm up
        moveArm();//bring the arm up
        sleep(1000);//sleep
        power_arm = 0;//keep the arm in place
        moveArm();//keep the arm in place
        elbow_power = -0.1;//tilt the elbow down to place
        moveElbow();//tilt the elbow down to place
        sleep(550);//sleep
        OpenBottomClaw();
        sleep(1000);//sleep
        elbow_power = 0.8;//bring elbow up
        moveElbow();//bring elbow up
        sleep(800);
        //Start of Park
        driveStraightPID(-11);//go backward
        sleep(500);//sleep
        power_arm = -10;//push the arm down
        moveArm();//push the arm down
        sleep(600);//sleep
        power_arm = 0;//keep the arm in place
        moveArm();//keep the arm in place
        driveStraightPID(-6);//go backward
        StrafingAUTO(20,false);//strafe to park area
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


