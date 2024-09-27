package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "TestingBlue25Farther", group = "Autonomous")
public class deep_auto_bluecloser extends RoboEaglesAutonomousBase {

    //public DcMotorEx armMotor;

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

        autonomousStartBlue();
        //telemetry.update();

        //waitForStart();
        //autonomousStartTest();
    }

    private void MapDevicesTesting() {
        //armMotor = hardwareMap.DcMotorEx.get("arm_motor");
        brDrive = hardwareMap.dcMotor.get("br_motor");
        blDrive = hardwareMap.dcMotor.get("bl_motor");
        flDrive = hardwareMap.dcMotor.get("fl_motor");
        frDrive = hardwareMap.dcMotor.get("fr_motor");
        brClaw = hardwareMap.servo.get("br_claw");
        blClaw = hardwareMap.servo.get("bl_claw");
        leftPower = 10;
        rightPower = 10;


    }
    private final double LEFT_SIDE_MULTIPLIER    = 1;
    private final double RIGHT_SIDE_MULTIPLIER   = 1;
    private long TimeToRun;
    private final int ONE_SECOND = 1000;
    private final double DEGREE_PER_SEC = 380 ;
    private final long go_straight_time_const = 25000;
    private final double DEGREE_PER_SEC_NEW = 44;
    public void driveStraightSmoothlyNew(long speed, long distance) {
        leftPower = -1 * (speed);
        //leftPower = -10;
        rightPower = leftPower;
        flDrive.setPower(leftPower);
        blDrive.setPower(leftPower);
        frDrive.setPower(rightPower * (-1));
        brDrive.setPower(rightPower * (-1));
        //TimeToRun = (distance * 10 / (84 * speed));
        TimeToRun = (distance * go_straight_time_const / (400 * speed));
        //sleep(500);
        sleep(TimeToRun);
        leftPower = 0;
        rightPower = 0;
        flDrive.setPower(leftPower);
        blDrive.setPower(leftPower);
        frDrive.setPower(rightPower * (-1));
        brDrive.setPower(rightPower * (-1));
    }

    public void Turning_Still(long TurnSpeed, long angle) {
        double absAngle = Math.abs (angle);						// Absolute value of the distance that will be using for all the calculations
        double altTurnSpeedLeft = TurnSpeed * LEFT_SIDE_MULTIPLIER;	// value of the speed for LEFT side motors adjusted by respective modifier
        double altTurnSpeedRight = TurnSpeed * RIGHT_SIDE_MULTIPLIER;	// value of the speed for RIGHT side motors adjusted by respective modifier


        double stopPower = 0.0;
        long baseTime = 1000; // base time for which robot turns 45 degrees at full speed
        long timeToTurn = 0;

        if (angle == 0.0) {
                // Zero angle means we are done
            return;
            }

        if (angle > 0.0) {
                // clockwise turn, we only move left side motors
                rightPower = altTurnSpeedRight * -1.0;
                leftPower = altTurnSpeedLeft;
            } else {
                // counter clockwise turn, we only move right side motors
                rightPower = altTurnSpeedRight;
                leftPower = altTurnSpeedLeft * -1.0;
            }

            // how do we calculate the time:
            // We use assumption that it turns 45 degree at full speed for 1000 milliseconds
            // we would need to confirm and may change baseTime to something else later
        timeToTurn = (long) Math.round(ONE_SECOND * (absAngle / DEGREE_PER_SEC) / altTurnSpeedLeft);

        //moveSidesSame(timeToTurn, leftPower, rightPower);
        moveSidesSame(2, leftPower, rightPower);

            //************************
            //completely stop
            //************************
            moveAllWheels(0,0);
        }
    public void Turning_New(long TurnSpeed, long angle) {
        double absAngle = Math.abs (angle);						// Absolute value of the distance that will be using for all the calculations
        double altTurnSpeedLeft = TurnSpeed * LEFT_SIDE_MULTIPLIER;	// value of the speed for LEFT side motors adjusted by respective modifier
        double altTurnSpeedRight = TurnSpeed * RIGHT_SIDE_MULTIPLIER;	// value of the speed for RIGHT side motors adjusted by respective modifier


        double stopPower = 0.0;
        long baseTime = 1000; // base time for which robot turns 45 degrees at full speed
        long timeToTurn = 0;

        if (angle == 0.0) {
            // Zero angle means we are done
            return;
        }

        if (angle > 0.0) {
            // clockwise turn, we only move left side motors
            rightPower = altTurnSpeedRight * -1.0;
            leftPower = altTurnSpeedLeft;
        } else {
            // counter clockwise turn, we only move right side motors
            rightPower = altTurnSpeedRight;
            leftPower = altTurnSpeedLeft * -1.0;
        }

        //leftPower = -10;
        //rightPower = -10;
        flDrive.setPower(leftPower);
        blDrive.setPower(leftPower);
        frDrive.setPower(rightPower * (-1));
        brDrive.setPower(rightPower * (-1));
        //TimeToRun = (distance * 10 / (84 * speed));
        //TimeToRun = (distance * 10000 / (400 * speed));
        telemetry.addData("Nitin running DrivingNormal", "my_val = %f",leftPower);
        //telemetry.update();
        //sleep(500);

        timeToTurn = (long) Math.round(ONE_SECOND * (absAngle / DEGREE_PER_SEC_NEW) / altTurnSpeedLeft);
        telemetry.addData("DrivingNormal", "Left Power 1: %f", leftPower);
        sleep(timeToTurn);
      //  telemetry.update();
        leftPower = 0;
        rightPower = 0;
        flDrive.setPower(leftPower);
        blDrive.setPower(leftPower);
        frDrive.setPower(rightPower * (-1));
        brDrive.setPower(rightPower * (-1));

    }



     void autonomousStartBlue() {
   /*
        //VERY IMPORTANT!!!!! OPEN=-0.5 and CLOSE=0.8!!!!!!!!!
        // Move to the Middle
         driveStraightSmoothlyNew(10,23); //if it is inches then it is 30
         telemetry.addData("DrivingNormal", "Left Power 2: %f", 0);
         sleep(500);
         Turning_New(10, 90);
         sleep(500);
         driveStraightSmoothlyNew(10, 32); //if it is inches then it is 31
         sleep(500);
         Turning_New(10, -90);
         sleep(500);
         driveStraightSmoothlyNew(10, 12); //if it is inches then it is 8
         //move arm and elbow
         sleep(500);
*/
        // Open the bottom claw to release the pixel
         /*brClaw.setPosition(0);
         sleep(2000);
         brClaw.setPosition(0.8);
         sleep(2000);
         brClaw.setPosition(0);
         sleep(2000);
         //brClaw.setPosition(0.2);*/
         blClaw.setPosition(0);
         sleep(2000);
         /*blClaw.setPosition(0.8);
         sleep(2000);
         blClaw.setPosition(0);
         sleep(2000);*/



    }

    void autonomousStartTest() {
        telemetry.addData("DrivingNormal", "Left Power 1: %f", 0);
        telemetry.update();
        driveStraightSmoothlyNew(40,5000000); //if it is inches then it is 30
        telemetry.addData("DrivingNormal", "Left Power 1D: %f", 0);
        telemetry.update();
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


