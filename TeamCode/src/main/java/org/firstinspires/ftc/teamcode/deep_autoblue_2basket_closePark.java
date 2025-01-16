package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "Basket2ClosePark", group = "Autonomous")
public  class deep_autoblue_2basket_closePark extends RoboEaglesAutoBase2425 {
    boolean close_farther = true;
    boolean no_park = false;
    boolean with_sample = true;
    @Override
    public void runOpMode() {

        MapDevicesTesting();
        CloseBaseClaw();
        OpenBottomClaw();
        power_arm = -10;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep;
        power_arm = 0;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep;

        // Enable the TFOD processor for our TeamProp Detection.
        while (!isStarted() && !isStopRequested()) {
            //   detectTeamProp();
            // Do all the other stuff
            //telemetry.update();
            sleep(10);
        }


        autonomousStartBlueBasket();
        sleep(500);
    }

    double distance;
    public void turnPID(int angle, int tolerance) {
        double speed_multiplier = 0.58;
        double angle_nonzer_subtract = 0;
        /*
        if (battery_power > 13) {
            speed_multiplier = 0.53;
        }
        if (battery_power > 13.75){
            speed_multiplier = 0.53;
            angle_nonzer_subtract = 0.53;
        }else if(battery_power > 13.5) {
            speed_multiplier = 0.55;
            angle_nonzer_subtract = 0.55;
        }else if (battery_power > 13.25) {
            speed_multiplier = 0.495;
            angle_nonzer_subtract = 0.495;
        } else if(battery_power > 13) {
            speed_multiplier = 0.51;
            angle_nonzer_subtract = 0.51;
        } else if (battery_power > 12.75) {
            speed_multiplier = 0.53 ;
            angle_nonzer_subtract = 0.53;
        } else if (battery_power > 12.50) {
            speed_multiplier = 0.55;
            angle_nonzer_subtract = 0.55;
        } else if (battery_power > 12.25) {
            speed_multiplier = 0.57;
            angle_nonzer_subtract = 0.57;
        } else if (battery_power > 12) {
            speed_multiplier = 0.58;
            angle_nonzer_subtract = 0.58;
        } else  {
            speed_multiplier = 0.45;
        }
        if (angle<0) {
            speed_multiplier = -angle_nonzer_subtract;
        }

         */
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
    void autonomousStartBlueBasket() {
        // first routine
        DRIVE_SPEED_MULTIPLIER = 0.5;
        StrafingAUTO(11, false);
        driveStraightPID(18);
        elbow_power = 4;
        moveElbow();
        sleep(200);
        turnPID(55,20);
        //driveStraightPID(5);
        drop_basket();
        //sleep(500); //SM
        turnPID(235,20);//235
        // turnPID(240,20);
        driveStraightPID(14);
        elbow_power = -0.3;
        moveElbow();
        sleep(1500); //1550
        CloseBaseClaw();
        sleep(1200);
        elbow_power = 1;
        moveElbow();
        sleep(1900);


        //start of delivering 2nd sample to the basket
        turnPID(170,20);
        driveStraightPID(11);
        power_arm = 27;
        moveArm();
        sleep(1400);
        power_arm = 0.05;//keep the arm in one place with almost no power
        moveArm();
        driveStraightPID(3);
        elbow_power = -0.5;
        moveElbow();
        sleep(300);
        OpenBaseClaw();
        sleep(1000);
        elbow_power = 4;
        moveElbow();
        sleep(500);//1000
        driveStraightPID(-4);
        sleep(100);
        power_arm = -12;//also used to be -10
        moveArm();
        sleep(1200);
        power_arm = 0;
        moveArm();
        sleep(500);
        //start of going to 3 point hang
        turnPID(80,20);
        strafe_power = 2;
        StrafingAUTO(70,true);
        driveStraightPID(10);
    }


    public void drop_basket() {
        // drop sample
        power_arm = 27;
        moveArm();
        sleep(2500);//sleep //2100 but we changed from 20:1 to 40:1
        power_arm = 0.05;//keep the arm in one place with almost no power
        moveArm();
        DRIVE_SPEED_MULTIPLIER = 0.65;
        driveStraightPID(4);
        elbow_power = -0.5;
        moveElbow();
        sleep(300);
        OpenBaseClaw();
        sleep(1000);

        elbow_power = 4;
        moveElbow();
        sleep(1000);

        driveStraightPID(-4);
        sleep(100);
        power_arm = -12;//also used to be -10
        moveArm();
        sleep(850);

        power_arm = 0;
        moveArm();
        sleep(500);



    }
    public void turntest()  {
        driveStraightPID(4);
        turnPID(-42,20);
        sleep(500);
    }

    long power_factor = 1000/25;

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


