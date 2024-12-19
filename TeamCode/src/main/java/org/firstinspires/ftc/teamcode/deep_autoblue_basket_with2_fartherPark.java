package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "XXXFBasketFartherPark", group = "Autonomous")
public class deep_autoblue_basket_with2_fartherPark  extends deep_autoblue_basket_with2_closePark{
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
        close_farther = false;
        autonomousStartBlueBasket();

    }
}
