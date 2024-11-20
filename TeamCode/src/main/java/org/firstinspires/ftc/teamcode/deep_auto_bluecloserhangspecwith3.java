package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "Blue25CloserHangSpecWITH3SAMPLES", group = "Autonomous")
public class deep_auto_bluecloserhangspecwith3 extends RoboEaglesAutoBase2425 {

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

        //autonomousStartBlueHangLower();
        autonomousStartBlueHangHigher(true);
        //telemetry.update();

        //waitForStart();
        //autonomousStartTest();
    }


    double distance;

    void autonomousStartBlueHangHigher(boolean blue_auto) {
        if (blue_auto == true) {
            turn_value = 1;

        } else {
            turn_value = -1;
        }
        elbow_power = -0.5;//put the elbow up
        moveElbow();//add in elbow function
        driveStraightPID(26); //move the robot 16 inches
        //sleep(1000);//sleep
        sleep(1000);//sleep
        power_arm = 10;//extend the arm up
        moveArm();//add in arm function
        sleep(600);//sleep
        power_arm = 0;//extend the arm up
        moveArm();//add in arm function
        elbow_power = 0;//put the elbow up
        moveElbow();//add in elbow function
        //Manish this was 500ms, we changed it to give it time to retract
        // this is working fine keep 1s.
        sleep(1000);//sleep
        // Manish changing the clow opening a bit more
        //this is because servo wires were swapped.
        brClaw.setPosition(0.7);//open right claw...also closing for this claw is 0.2
        blClaw.setPosition(0.2);//open left claw...also closing for this claw is 0.7
        //Original code
        //brClaw.setPosition(0.7);//open right claw...also closing for this claw is 0.2
        //blClaw.setPosition(0.2);//open left claw...also closing for this claw is 0.7
        //Manish code starts
        sleep(1000);//sleep
        elbow_power = -0.8;
        moveElbow();
        //Manish code ends here
        //sleep(2000);//sleep
        //elbow_power = 0.4;//put the elbow up
        //moveElbow();//add in elbow function
        sleep(500);//sleep
        driveStraightPID(-12);
        sleep(500);//sleep
        power_arm = -10;//extend the arm up
        moveArm();//add in arm function
        sleep(600);//sleep
        StrafingAUTO(34,true);
        driveStraightPID(39);
        StrafingAUTO(8,true);
        driveStraightPID(-48);
        driveStraightPID(50);
        StrafingAUTO(8,true);
        driveStraightPID(-40);
        driveStraightPID(40);
        StrafingAUTO(6,true);
        driveStraightPID(-35);
        //final_park_hang(close_farther);
        sleep(1500);
    }

    public void final_park_hang(boolean close) {
        if (close) {

            turnPID(-90, 20);
            driveStraightPID(28);
            turnPID(-90, 10);
            driveStraightPID(12);
        } else {
            turnPID(-90, 20);
            driveStraightPID(46);
            turnPID(-90, 10);
            driveStraightPID(12);
        }
    }
}


