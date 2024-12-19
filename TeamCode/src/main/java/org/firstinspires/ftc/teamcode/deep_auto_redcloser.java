package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "XXRed25CloserBasket", group = "Autonomous")
public class deep_auto_redcloser extends deep_auto_bluecloser {

    //public DcMotorEx armMotor;

    @Override
    public void runOpMode() {

        MapDevicesTesting();
        waitForStart();
        autonomousStartBlueBasket(false);
    }



    @Override
    void placePurplePixelLeft() {

    }

    @Override
    void placePurplePixelMiddle() {

    }

    @Override
    void placePurplePixelRight() {



    }
}


