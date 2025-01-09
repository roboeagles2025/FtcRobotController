package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "TurningImprovements", group = "Autonomous")
public class Turning_Improvements extends RoboEaglesAutoBase2425 {

    boolean close_farther = true;
    boolean no_park = false;
    boolean with_sample = true;
    boolean close_simple = true;
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
        //autonomousStartBlueHangHigher();
        //telemetry.update();
        practice_turning();
        sleep(500);
        telemetry.update();
        //waitForStart();
        //autonomousStartTest();
    }
    double distance;
    public void turnPID2(int angle, int tolerance) {
        double speed_multiplier = 0.57;
        if (angle<0) {
            speed_multiplier = -0.57;
        }
        flDriveEx.set(-speed_multiplier);
        frDriveEx.set(speed_multiplier*1.5);
        blDriveEx.set(-speed_multiplier);
        brDriveEx.set(speed_multiplier*1.5);
        sleep(Math.abs(angle)*6);
        flDriveEx.set(0);
        frDriveEx.set(0);
        blDriveEx.set(0);
        brDriveEx.set(0);
        sleep(500);
    }
    public void drivemove() {
        double speed_multiplier = 0.275;
        flDriveEx.resetEncoder();
        blDriveEx.resetEncoder();
        brDriveEx.resetEncoder();
        frDriveEx.resetEncoder();
        telemetry.addData("Drive PID", "FL: %d, Fr: %d, Bl: %d, Br: %d", flDriveEx.getCurrentPosition(), frDriveEx.getCurrentPosition(), blDriveEx.getCurrentPosition(), brDriveEx.getCurrentPosition());
        flDriveEx.set(speed_multiplier);
        frDriveEx.set(speed_multiplier);
        blDriveEx.set(speed_multiplier);
        brDriveEx.set(speed_multiplier);
        sleep(6000);
        flDriveEx.set(0);
        frDriveEx.set(0);
        blDriveEx.set(0);
        brDriveEx.set(0);
        sleep(500);
    }
    void practice_movement() {
        drivemove();
        telemetry.addData("Drive PID", "FL: %d, Fr: %d, Bl: %d, Br: %d", flDriveEx.getCurrentPosition(), frDriveEx.getCurrentPosition(), blDriveEx.getCurrentPosition(), brDriveEx.getCurrentPosition());
    }
    void practice_turning() {
        //for (int turn = 0; turn < 12;turn++) {
        strafe_power = 2;
        StrafingAUTO(50,false);
        //}
    }

}


