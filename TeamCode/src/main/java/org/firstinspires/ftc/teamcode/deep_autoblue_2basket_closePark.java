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
        //brClaw.setPosition(0.43);//NEVER CHANGE THIS CODE!!!!!!!
        //blClaw.setPosition(0);//0.45
        OpenBottomClaw();


        power_arm = -10;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep;
        power_arm = 0;//push down the arm to properly pick up the specimen from the zone
        moveArm();//push down the arm to properly pick up the specimen from the zone
        sleep(100);//sleep;
        battery_volt = hardwareMap.voltageSensor.iterator().next();
        battery_power = battery_volt.getVoltage();
        telemetry.addData("Battery voltage = %f", battery_power);
        telemetry.update();
        // Enable the TFOD processor for our TeamProp Detection.
        while (!isStarted() && !isStopRequested()) {
            //   detectTeamProp();
            // Do all the other stuff
            //telemetry.update();
            sleep(10);
        }


        autonomousStartBlueBasket_leftstraf();
        //turnTests();
        //clawTest();
        sleep(500);
    }

    public void turnTests(){
        DRIVE_SPEED_MULTIPLIER = 0.7;
        for (int k=0; k<4;k++) {
            turnPID(45, 40);
            sleep(500);
            driveStraightPID(6);
            sleep(500);
        }
    }

    double distance;
    int offset;
    public void turnPID(int angle, int tolerance) {
        turnWoPID(angle);
    }

  /*  void autonomousStartBlueBasket() {
        // first routine
        DRIVE_SPEED_MULTIPLIER = 0.5;
        StrafingAUTO(11, false);
        driveStraightPID(18);
        elbow_power = 4;
        moveElbow();
        sleep(200);
        turnPID(65,20);
        //driveStraightPID(5);
        drop_basket();

        // start of picking second sample
        //sleep(500); //SM
        turnPID(267,20);//235
        // turnPID(240,20);
        driveStraightPID(15);
        elbow_power = -0.3;
        moveElbow();
        sleep(1500); //1550
        CloseBaseClaw();
        sleep(1200);
        elbow_power = 1;
        moveElbow();
        sleep(1900);


        //start of delivering 2nd sample to the basket
        turnPID(200,20);
        driveStraightPID(13);
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
*/
    void autonomousStartBlueBasket_leftstraf() {
        // first routine
        DRIVE_SPEED_MULTIPLIER = 0.5;
        StrafingAUTO(10, true);
        driveStraightPID(-15);
        elbow_power = 4;
        moveElbow();
        sleep(200);
        turnPID(-165,20);
        driveStraightPID(4);
        drop_basket();

        // start of picking second sample
        //sleep(500); //SM
        turnPID(270,20);//235
        // turnPID(240,20);
        driveStraightPID(14);
        elbow_power = -0.3;
        moveElbow();
        sleep(1400); //1550
        CloseBaseClaw();
        sleep(1000);
        elbow_power = 1;
        moveElbow();
        sleep(1000);//1900


        //start of delivering 2nd sample to the basket
        driveStraightPID(-13.5);
        turnPID(152,20);
        driveStraightPID(3);
        power_arm = 27;
        moveArm();
        sleep(1400);
        power_arm = 0.1;//keep the arm in one place with almost no power
        moveArm();
        driveStraightPID(3);
        elbow_power = -0.5;
        moveElbow();
        sleep(300);
        OpenBaseClaw();
        sleep(1000);
        elbow_power = 100; //4
        moveElbow();
        sleep(1000);//1000
        driveStraightPID(-4);
        sleep(100);
        power_arm = -40;//also used to be -10
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

        elbow_power = 100;
        moveElbow();
        sleep(1200);

        driveStraightPID(-5);
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


