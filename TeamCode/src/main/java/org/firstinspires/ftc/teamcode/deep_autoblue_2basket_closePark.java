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
        if (angle<0) {
            speed_multiplier = -0.58;
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
    void autonomousStartBlueBasket() {
        // first routine

        StrafingAUTO(11, false);
        driveStraightPID(18);
        turnPID(45,20);
        //driveStraightPID(5);
        drop_basket();
        sleep(500);
        turnPID(-150,20);
        driveStraightPID(10);
        elbow_power = -0.1;
        moveElbow();
        sleep(1400);
        brClaw.setPosition(0);
        blClaw.setPosition(0.45);
        sleep(1200);
        elbow_power = 4;
        moveElbow();
        sleep(500);


        //start of delivering 2nd sample to the basket
        turnPID(168,20);
        driveStraightPID(10);
        power_arm = 27;
        moveArm();
        sleep(1200);
        power_arm = 0.05;//keep the arm in one place with almost no power
        moveArm();
        elbow_power = -0.5;
        moveElbow();
        sleep(300);
        brClaw.setPosition(0.45);
        blClaw.setPosition(0);
        sleep(1000);
        elbow_power = 4;
        moveElbow();
        sleep(1000);
        driveStraightPID(-4);
        sleep(100);
        power_arm = -12;//also used to be -10
        moveArm();
        sleep(800);
        power_arm = 0;
        moveArm();
        sleep(500);
    }


    public void drop_basket() {
        // drop sample
        elbow_power = 4;
        moveElbow();
        sleep(200);
        power_arm = 27;
        moveArm();
        sleep(2100);//sleep //2100 but we changed from 20:1 to 40:1
        power_arm = 0.05;//keep the arm in one place with almost no power
        moveArm();
        DRIVE_SPEED_MULTIPLIER = 0.65;
        driveStraightPID(4);
        elbow_power = -0.5;
        moveElbow();
        sleep(300);
        brClaw.setPosition(0.45);
        blClaw.setPosition(0);
        sleep(1000);

        elbow_power = 4;
        moveElbow();
        sleep(1000);

        driveStraightPID(-4);
        sleep(100);
        power_arm = -12;//also used to be -10
        moveArm();
        sleep(650);

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


