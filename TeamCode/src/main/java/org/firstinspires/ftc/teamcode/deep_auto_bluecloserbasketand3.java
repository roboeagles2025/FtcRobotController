package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "Blue25CloserBasketSample2", group = "Autonomous")
public  class deep_auto_bluecloserbasketand3 extends RoboEaglesAutoBase2425 {
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
        //bucketand2();
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
        //driveStraightPID(-15);//drive backwards 20 inches
        turnPID(45,20);//used to be 45

        //start of 3 samples in net zone
        StrafingAUTO(12,true);//used to be 8
        driveStraightPID(-32);
        StrafingAUTO(12,false);//used to be 8
        driveStraightPID(47);
        //StrafingAUTO(3,false);
        driveStraightPID(-50);
        StrafingAUTO(8,false);
        driveStraightPID(45);
        driveStraightPID(-15);
        // second routine

        /*power_arm = -10;//detract the arm down
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
        brClaw.setPosition(0.2); // closed
        blClaw.setPosition(0.7);
        sleep(1000);
        power_arm = -10;//detract the arm down
        moveArm();//add in arm function
        sleep(1300);//sleep

        turnPID(150,20);
        driveStraightPID(6);
        drop_basket();

        sleep(1300);//sleep

         */
    }


    public void drop_basket() {
        // drop sample
        elbow_power = -4;//put the elbow up
        moveElbow();
        sleep(500);//sleep
        power_arm = 15;//extend the arm up
        moveArm();
        sleep(1800);//sleep
        power_arm = 0.05;//keep the arm in one place with almost no power
        moveArm();
        driveStraightPID(13);
        elbow_power = 0;//put the elbow up
        moveElbow();//add in elbow function
        sleep(500);//sleep
        brClaw.setPosition(0.7);//open right claw...also closing for this claw is 0.2
        blClaw.setPosition(0.2);//open left claw...also closing for this claw is 0.7
        sleep(1000);//sleep

        elbow_power = -4;//put the elbow up
        moveElbow();//add in elbow function
        sleep(500);//sleep
        driveStraightPID(-16);//drive backwards a feet
        power_arm = -12;//detract the arm down it also used to be -10
        moveArm();//add in arm function
        sleep(800);//sleep it used to be 1 second
        power_arm = 0;//set to 0 power for no movements
        moveArm();//add in arm function
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


