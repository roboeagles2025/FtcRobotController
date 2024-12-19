package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "FHangSpecNoSampParkFarther", group = "Autonomous")
public class deep_autoblue_hangspec_nosamp_justparkfarther extends deep_autoblue_hangspec_with2_closePark {



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
        close_simple = false;
        autonomousStartBlueHangHigher();
        //telemetry.update();

        //waitForStart();
        //autonomousStartTest();
    }




}


