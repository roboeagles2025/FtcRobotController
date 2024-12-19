package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "FBasketClosePark", group = "Autonomous")
public  class deep_autoblue_basket_with2_closePark extends RoboEaglesAutoBase2425 {
    boolean close_farther = true;
    boolean no_park = false;
    boolean with_sample = true;
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


        autonomousStartBlueBasket();

    }


    void autonomousStartBlueBasket() {
        // first routine

        StrafingAUTO(11, false);
        driveStraightPID(18);
        turnPID(47,20);
        //driveStraightPID(5);
        drop_basket();
        if(with_sample) {
            DRIVE_SPEED_MULTIPLIER = 0.75;
            //driveStraightPID(-15);//drive backwards 20 inches
            turnPID(47, 10);//used to be 45


            //start of 2 samples in net zone
            StrafingAUTO(9, true);
            driveStraightPID(-32);
            StrafingAUTO(9, false);

            driveStraightPID(43);

            driveStraightPID(-42);
            StrafingAUTO(7, false);
            driveStraightPID(38);
            //
            driveStraightPID(-12);
            StrafingAUTO(56, true);

            //parking time
        /*
        if(no_park == false) {
            final_park_hang(close_farther);
        }
    */
        }
        sleep(500);
    }


    public void drop_basket() {
        // drop sample
        elbow_power = -4;
        moveElbow();
        sleep(200);
        power_arm = 27;
        moveArm();
        sleep(2100);//sleep //2100 but we changed from 20:1 to 40:1
        power_arm = 0.05;//keep the arm in one place with almost no power
        moveArm();
        DRIVE_SPEED_MULTIPLIER = 0.65;
        driveStraightPID(10);
        elbow_power = 0.5;
        moveElbow();
        sleep(300);
        blClaw.setPosition(0.45);
        brClaw.setPosition(0);
        sleep(1000);

        elbow_power = -4;
        moveElbow();
        sleep(1000);

        driveStraightPID(-16);
        sleep(100);
        power_arm = -12;//also used to be -10
        moveArm();
        sleep(1500);

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
    public void StrafingAUTO(long distance, boolean turn) {
        double  strafe_power = 0.5;
        if (turn == false) {
            strafe_power = -0.5;
        }

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


