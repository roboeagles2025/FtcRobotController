package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "FHangSpecNoSampParkClose", group = "Autonomous")
public class deep_autoblue_hangspec_nosamp_justpark extends deep_autoblue_hangspec_with2_closePark {

    boolean close_farther = true;
    boolean no_park = false;

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
        with_sample = false;
        close_simple = true;
        autonomousStartBlueHangHigher();
        //telemetry.update();

        //waitForStart();
        //autonomousStartTest();
    }


    double distance;


}


