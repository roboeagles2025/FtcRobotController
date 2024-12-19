package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "TestingTime", group = "Autonomous")
public class Test_Increasing_AutoTIME extends RoboEaglesAutoBase2425 {

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
        TestingAutoTime();
        //telemetry.update();

        //waitForStart();
        //autonomousStartTest();
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


