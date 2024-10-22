package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "TestingRed25Closer", group = "Autonomous")
public class deep_auto_redcloser extends RoboEaglesAutonomousBase {

    //public DcMotorEx armMotor;

    @Override
    public void runOpMode() {

        MapDevicesTesting();
        waitForStart();
        autonomousStartRed();
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

    public void driveStraightSmoothlyNew(long speed, long distance) {
        leftPower = -10;
        rightPower = -10;
        flDrive.setPower(leftPower);
        blDrive.setPower(leftPower);
        frDrive.setPower(rightPower * (-1));
        brDrive.setPower(rightPower * (-1));
        TimeToRun = (distance * 10 / (84 * speed));
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

        moveSidesSame(timeToTurn, leftPower, rightPower);

            //************************
            //completely stop
            //************************
            moveAllWheels(0,0);
        }


     void autonomousStartRed() {
        // Move to the Middle
        //driveStraightSmoothly(0.5, 230);
         driveStraightSmoothlyNew(10,76); //if it is inches then it is 30
         sleep(2000);
         Turning_Still(10, 90);
         sleep(2000);
         /*
         driveStraightSmoothlyNew(10, 79); //if it is inches then it is 31
         Turning_Still(10, -90);
         driveStraightSmoothlyNew(10, 20); //if it is inches then it is 8
         //move arm and elbow
         Turning_Still(10, -135);
         driveStraightSmoothlyNew(10, 61); //if it is inches then it is 24
         //move arm and elbow
         Turning_Still(10, 60);
         driveStraightSmoothlyNew(10, 63); //if it is inches then it is 25
         //use bottom claw
         Turning_Still(10, -150);
         driveStraightSmoothlyNew(10, 43); //if it is inches then it is 17
         //move arm and elbow
         Turning_Still(10, 120);
         driveStraightSmoothlyNew(10, 261); //if it is inches then it is 103
         //end of auto red closer to the basket!!!



        //turnPID(90, 0.5);
        //driveStraightSmoothly(0.5, 32);
        //turnPID(90, 0.5);
        //driveStraightSmoothly(0.5, 22.25);

        //armMotor.setPower(10);
        //sleep(10);
        //armMotor.setPower(0);

        // Open the bottom claw to release the pixel
        blClaw.setPosition(BOTTOM_LEFT_CLAW_OPEN);
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_OPEN);
        sleep(1000);

          */
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


