package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.concurrent.*;
@Autonomous(name = "TestingTime", group = "Autonomous")
public class Test_Increasing_AutoTIME extends RoboEaglesAutoBase2425 {
    boolean sync = false;
    @Override
    public void runOpMode() {
        final ExecutorService pool = Executors.newFixedThreadPool(2);

        MapDevicesTesting();
        // Enable the TFOD processor for our TeamProp Detection.
        while (!isStarted() && !isStopRequested()) {
            //   detectTeamProp();
            // Do all the other stuff
            //telemetry.update();
            sleep(10);
        }
        //waitForStart();
        StrafingAUTO(11, false);
        driveStraightPID(14);
        turnPID(47,20);
        sync = true;


        elbow_power = 4;
        moveElbow();
        sleep(200);
        power_arm = 27;
        moveArm();
        sleep(2100);//sleep //2100 but we changed from 20:1 to 40:1
        power_arm = 0.05;//keep the arm in one place with almost no power
        moveArm();
        DRIVE_SPEED_MULTIPLIER = 0.65;
        while(sync == false)
        {
            sleep(10);
        }
        driveStraightPID(10);
        drop_basket();

        //autonomousStartBlueHangLower();
        //TestingAutoTime();

        /*
        pool.execute(() -> {
            StrafingAUTO(11, false);
            driveStraightPID(14);
            turnPID(47,20);
            sync = true;
        });
        pool.execute(() -> {
            elbow_power = 4;
            moveElbow();
            sleep(200);
            power_arm = 27;
            moveArm();
            sleep(2100);//sleep //2100 but we changed from 20:1 to 40:1
            power_arm = 0.05;//keep the arm in one place with almost no power
            moveArm();
            DRIVE_SPEED_MULTIPLIER = 0.65;
            while(sync == false)
            {
                sleep(10);
            }
            driveStraightPID(10);
            drop_basket();
        });
    */

        //driveStraightPID(5);

        sleep(50000);
        //telemetry.update();

        //waitForStart();
        //autonomousStartTest();
    }

    public void drop_basket() {
        // drop sample
        /*
        elbow_power = 4;
        moveElbow();
        sleep(200);
        power_arm = 27;
        moveArm();
        sleep(2100);//sleep //2100 but we changed from 20:1 to 40:1
        power_arm = 0.05;//keep the arm in one place with almost no power
        moveArm();
        DRIVE_SPEED_MULTIPLIER = 0.65;
        driveStraightPID(10);
        */

        //elbow_power = -0.5;
        //moveElbow();
        //sleep(300);
        blClaw.setPosition(0.45);
        brClaw.setPosition(0);
        sleep(1000);

        elbow_power = 4;
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
    double distance;

    void TestingAutoTime() {
            driveStraightPID(20);//drives straight for 20 inches
            power_arm = 10;//
            moveArm();
            sleep(1000);
            power_arm = -10;
            moveArm();
            sleep(1000);
    }
    void repeattest() {
        for ( int repeat=0; repeat > 20 ; repeat++) {
            driveStraightPID(1);
            power_arm = 10;
            moveArm();
            sleep(50);
            power_arm = -10;
            moveArm();
            sleep(50);
        }
    }

    /*void main(final String[] args) throws InterruptedException {
        final ExecutorService pool = Executors.newFixedThreadPool(2);
        pool.execute(());
    }*/

}


