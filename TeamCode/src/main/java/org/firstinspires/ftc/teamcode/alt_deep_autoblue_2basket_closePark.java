package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "AltBasket2ClosePark", group = "Autonomous")
public  class alt_deep_autoblue_2basket_closePark extends RoboEaglesAutoBase2425 {
    boolean close_farther = true;
    boolean no_park = false;
    boolean with_sample = true;

    @Override
    public void runOpMode() {
        battery_volt = hardwareMap.voltageSensor.iterator().next();

        MapDevicesTesting();
        CloseBaseClaw();
        OpenBottomClaw();
        power_arm = -10;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep;
        power_arm = 0;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep;
        elbow_power = 1;//lift up elbow to get ready for dropping
        moveElbow();//lift up elbow to get ready for dropping
        // Enable the TFOD processor for our TeamProp Detection.
        while (!isStarted() && !isStopRequested()) {
            //   detectTeamProp();
            // Do all the other stuff
            //telemetry.update();
            sleep(10);
        }

        //turns();
        autonomousStartBlueBasket();
        sleep(500);
    }


    void turns() {
        for (int i = 0; i < 16; i++) {


            turnPID_central(90, 20);
            sleep(1000);
        }
  /*      for (int j = 0; j < 8; j++) {
            turnPID(45, 20);
        }
        for (int k = 0; k < 10; k++) {
            turnPID(36, 20);
        }

        turnPID(170,10);
        turnPID(190,10);
        turnPID(50,10);
        turnPID(50,10);
        turnPID(50,10);
        turnPID(50,10);
        turnPID(50,10);
        turnPID(50,10);
        turnPID(50,10);
        turnPID(10,10);*/
    }

    void autonomousStartBlueBasket() {
        // first routine
        /* ORIGINAL CODE
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
        turnPID_central(235,20);//235
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
        */


        //Saket here: This the code for our basket routine.
        DRIVE_SPEED_MULTIPLIER = 0.5;

        StrafingAUTO(12, false);
        driveStraightPID(20);
        elbow_power = 4;
        moveElbow();
        turnPID_central(55, 20);
        drop_basket_NEW();
        turnPID_central(225, 20);//235
        driveStraightPID(1.5);
        elbow_power = -0.25;
        moveElbow();
        sleep(200); //1550
        elbow_power = 0.05;
        moveElbow();
        sleep(1500);
        CloseBaseClaw();
        sleep(1200);
        power_arm = -0.5;
        moveArm();
        sleep(200);
        elbow_power = 40;
        moveElbow();
        sleep(1200);


        //start of delivering 2nd sample to the basket
        turnPID_central(160, 20);
        //driveStraightPID(7);
        power_arm = 27;
        moveArm();
        sleep(2200);
        power_arm = 0.05;//keep the arm in one place with almost no power
        moveArm();
        driveStraightPID(10);
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
        sleep(200);
        power_arm = 0;
        moveArm();
        sleep(500);

        // working routine for straf and touch the rung. wthin 30 sec.
        /*turnPID_central(90,20);
        StrafingAUTO(72, false);*/

        //start of delivering 3rd sample to the basket
        turnPID_central(25, 20);
        DRIVE_SPEED_MULTIPLIER = 0.80;
        StrafingAUTO(14, false);
        //turnPID_central(5,20);
        driveStraightPID(-40);
        StrafingAUTO(16, false);
        driveStraightPID(36);

        StrafingAUTO(24, false);
        driveStraightPID(-32);
        turnPID_central(90, 20);
        driveStraightPID(24);

        //start of going to 3 po1int hang

        // turnPID_central(80,20);
        // strafe_power = 2;
        // StrafingAUTO(70,true);
        // driveStraightPID(10);
    }

    public void drop_basket_NEW() {
        // drop sample
        driveStraightPID(6);
        power_arm = 27;
        moveArm();
        sleep(2500); //2100 but we changed from 20:1 to 40:1
        power_arm = 0.05;//keep the arm in one place with almost no power
        moveArm();
        //DRIVE_SPEED_MULTIPLIER = 0.65;
        elbow_power = -0.5;
        moveElbow();
        sleep(300);
        OpenBaseClaw();
        sleep(500);

        elbow_power = 80;
        moveElbow();
        sleep(1000);

        driveStraightPID(-8);
        //sleep(100);
        power_arm = -12;//also used to be -10
        moveArm();
        sleep(200);

        power_arm = 0;
        moveArm();
        sleep(200);


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




