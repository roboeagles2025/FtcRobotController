package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "AutoBasketSENSORPark", group = "Autonomous")
public  class deep_autoblue_2basketSENSOR_closePark extends RoboEaglesAutoBase2425 {
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

        /*first_sample();
        //straightenup_robot();
        pick_sample(155,1300);
        straightenup_robot_second();
        pick_sample(190,1800);*/
        SENSORbasket();

        // LM3 CODE EXECUTES AS BELOW COMMENTED CODE.
        //first_sample();
        //second_sample();
        //third_sample();
        //sleep(500);
    }

    void straightenup_robot() {
        StrafingFAST(6,false);
        turnPID_central(205, 20);//225
        driveStraightPID_timer(500,-0.6);
        sleep(100);
        DRIVE_SPEED_MULTIPLIER = 0.65;
        driveStraightPID(11);
    }

    void straightenup_robot_second() {
        StrafingFAST(4,true);
        turnPID_central(220, 20);//225
        driveStraightPID_timer(600,-0.6);
        sleep(100);
        DRIVE_SPEED_MULTIPLIER = 0.85;
        driveStraightPID(12);
        StrafingFAST(7,true);
    }

    void SENSORbasket (){
        //Beginning of Code : TASK - Put Sample in Basket
        DRIVE_SPEED_MULTIPLIER = 0.65;
        elbow_power = 1;
        moveElbow();
        sleep(100);
        //driveStraightPID(7);
        power_arm = 0.85; // was not here in working condition
        moveArm(); // was not here in working condition
        StrafingAUTO(12, false); // was 12 in working condition
        DRIVE_SPEED_MULTIPLIER = 0.85;
        driveStraightPID(20.5);
        elbow_power = 4;
        moveElbow();
        turnPID_central(55, 20);
        drop_basket_Sensor();
    }
public void drop_basket_Sensor(){
    driveStraightPID(6);
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
    sleep(200);

    elbow_power = 80;
    moveElbow();
    sleep(500);
    DRIVE_SPEED_MULTIPLIER = 0.4;
    driveStraightPID(-5);
    DRIVE_SPEED_MULTIPLIER = 0.65;
    //sleep(100);
    power_arm = -12;//also used to be -10
    moveArm();
    sleep(100);

    power_arm = 0;
    moveArm();
    sleep(100);
}

        /*O BE REMOVED IF DOES NOT WORK 3rd sample
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
*/
}


