package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "AutoBasketPark", group = "Autonomous")
public  class deep_autoblue_2basket_Park extends RoboEaglesAutoBase2425 {
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

        // Enable the TFOD processor for our TeamProp Detection.
        while (!isStarted() && !isStopRequested()) {
            //   detectTeamProp();
            // Do all the other stuff
            //telemetry.update();
            sleep(100);
        }
        first_sample();
        second_sample();
        haul_sample();
        //parking();
        sleep(500);
    }





    public void haul_sample() {
        // working routine for straf and touch the rung. wthin 30 sec.
        /*turnPID_central(90,20);
        StrafingAUTO(72, false);*/
        //start of delivering 3rd sample to the basket
        turnPID_central(222, 20);// was 232
        DRIVE_SPEED_MULTIPLIER = 0.75;
        StrafingAUTO(12, true);
        //turnPID_central(5,20);
        driveStraightPID(46);
        StrafingFAST(14, false);
        //turnPID_central(5,20);
        //turnPID_central(-5,20);
        DRIVE_SPEED_MULTIPLIER = 0.85;
        driveStraightPID(-42);

    }
    public void parking(){
        driveStraightPID(4);
        StrafingFAST(16, false);
        driveStraightPID(40);

        turnPID_central(-90,20);
        //StrafingFAST(30, true);
        driveStraightPID(16);

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


