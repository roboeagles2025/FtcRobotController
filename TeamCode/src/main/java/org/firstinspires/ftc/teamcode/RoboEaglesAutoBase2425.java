package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

public class RoboEaglesAutoBase2425 extends RoboEaglesAutonomousBase {
    //public DcMotorEx armMotor;
    MotorGroup lGroup, rGroup;
    public com.arcrobotics.ftclib.controller.PIDController pidDriveLeft, pidDriveRight, pidRotate;
    com.arcrobotics.ftclib.controller.PIDController pidArm;
    RevIMU gyro;
    public double DRIVE_SPEED_MULTIPLIER = 0.75;

    MotorEx brDriveEx, blDriveEx, flDriveEx, frDriveEx;
    private final double LEFT_SIDE_MULTIPLIER    = 1;
    private final double RIGHT_SIDE_MULTIPLIER   = 1;
    public DcMotor leftElbow, rightElbow;
    public DcMotorEx rightArm;
    private long TimeToRun;
    private final int ONE_SECOND = 1000;
    private final double DEGREE_PER_SEC = 380 ;
    private final long go_straight_time_const = 25000;
    private final double DEGREE_PER_SEC_NEW = 44;
    public double power_arm, current_arm_pos;
    public Servo bottomrClaw;
    public Servo bottomlClaw;
    double elbow_power = 5;
    int turn_value = 1;
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

    }

    public void MapDevicesTesting() {
        //armMotor = hardwareMap.DcMotorEx.get("arm_motor");
        flDriveEx = new MotorEx(hardwareMap, "fl_motor");
        frDriveEx = new MotorEx(hardwareMap, "fr_motor");
        blDriveEx = new MotorEx(hardwareMap, "bl_motor");
        brDriveEx = new MotorEx(hardwareMap, "br_motor");
        brClaw = hardwareMap.servo.get("br_claw");
        blClaw = hardwareMap.servo.get("bl_claw");
        bottomrClaw = hardwareMap.servo.get("specl_claw");//OFFICIAL
        bottomlClaw = hardwareMap.servo.get("specr_claw");//OFFICIAL
        leftPower = 10;
        rightPower = 10;
        leftElbow = hardwareMap.dcMotor.get("left_elbow");//OFFICIAL
        rightElbow = hardwareMap.dcMotor.get("right_elbow");//OFFICIAL
        rightArm = hardwareMap.get(DcMotorEx.class, "right_arm");//OFFICIAL
        //rightElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//OFFICIAL
        //leftElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//OFFICIAL
        MapDevicesPIDs();

    }
    public void RoboEagleTurn(int angle){
        double speed_multiplier = 0.5;
        flDriveEx.set(-speed_multiplier);
        frDriveEx.set(speed_multiplier*1.5);
        blDriveEx.set(-speed_multiplier);
        brDriveEx.set(speed_multiplier*1.5);
        sleep(angle*6);
        flDriveEx.set(0);
        frDriveEx.set(0);
        blDriveEx.set(0);
        brDriveEx.set(0);
    }
    // mappng for all PID required hardware
    public void MapDevicesPIDs(){
        gyro = new RevIMU(hardwareMap);
        gyro.init();

        // Group left motors and right motors together
        lGroup = new MotorGroup(flDriveEx, blDriveEx);
        rGroup = new MotorGroup(frDriveEx, brDriveEx);

        // Reset all encoders to 0
        lGroup.stopAndResetEncoder();
        rGroup.stopAndResetEncoder();

        // Set left and right motor group to velocity control
        lGroup.setRunMode(Motor.RunMode.RawPower);
        rGroup.setRunMode(Motor.RunMode.RawPower);
        lGroup.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rGroup.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Reverse left motors
        lGroup.setInverted(true);

        pidArm = new com.arcrobotics.ftclib.controller.PIDController(0.005, 0.0001, 0);
        pidArm.setTolerance(10, 10);

        // Set the coefficients for the velocity PID controller - All values were taken from the docs
//        lGroup.setVeloCoefficients(0.05, 0.01, 0.31);
//        rGroup.setVeloCoefficients(0.05, 0.01, 0.31);

        // Set the coefficients for the feedforward controller - all values were taken from the docs
//        lGroup.setFeedforwardCoefficients(0.92, 0.47, 0.3);
//        rGroup.setFeedforwardCoefficients(0.92, 0.47, 0.3);

        // Initialize the PID controllers for the left motor group and right motor group
        // P - Proportional - Figured out through trial and error
        // I - Integral - "Impatience" - Will increase speed if error hasn't changed over time
        // D - Derivative - Will counteract proportional and integral - Disabled for now
        pidDriveLeft = pidDriveRight = new com.arcrobotics.ftclib.controller.PIDController(0.007, 0.0001, 0);
        pidDriveLeft.setTolerance(30, 20);
        pidDriveRight.setTolerance(30, 20);

        // Initialize the PID controller for turning
       // pidRotate = new PIDController(.006, .00008, 0.000001);
        pidRotate = new PIDController(.006, .00005, 0.000000);

    }

    void moveElbow() {
        // double elbow_power = 5; // Read the Y-axis value of the left joystick and negate it

        leftElbow.setPower(-elbow_power);
        rightElbow.setPower(-elbow_power);

    }

    void moveArm() {

        rightArm.setTargetPosition(40);
        //armMotor.setTargetPosition(armMotor.getTargetPosition() + (int) (power * 5));
        //leftArm.setPower(power);
        rightArm.setPower(-power_arm);


    }

    long power_factor = 1000/25;
    public void StrafingAUTO(long distance, boolean turn) {
        double  strafe_power = 1;
        if (turn == false) {
            strafe_power = -1;
        }
        long speed_multiplier = distance * power_factor;
        flDriveEx.set(-strafe_power);
        frDriveEx.set(strafe_power);
        blDriveEx.set(strafe_power);
        brDriveEx.set(-strafe_power);
        sleep(speed_multiplier);
        flDriveEx.set(0);
        frDriveEx.set(0);
        blDriveEx.set(0);
        brDriveEx.set(0);
        sleep(500);

    }

    public void driveStraightPID(double distance) {
        /*
        I don't think that we need the correction PID because as we are tracking the distance traveled by both the left set of wheels
        and the right set of wheels, if the robot starts to turn, the encoder values will differ and will automatically be corrected
         */
        lGroup.resetEncoder();
        rGroup.resetEncoder();
        gyro.reset();
        pidDriveLeft.reset();
        pidDriveRight.reset();
        int targetTicks = inchesToEncoderTicksInt(distance);
        pidDriveLeft.setSetPoint(targetTicks);
        pidDriveRight.setSetPoint(targetTicks);
        // I'm not sure whether FTCLib will be able to convert distance to encoder ticks,
        // If it doesn't work we can move over the encoderTicksToInches and inchesToEncoderTicks functions
        double speed = DRIVE_SPEED_MULTIPLIER; // Base speed that will be multiplied by the error
        // Previously used FTCLib's PositionControl, but that won't work as it isn't a PID Controller, only a PController,
        // so it gets stuck at lower speeds that the integral would have solved
        while (!pidDriveLeft.atSetPoint() && !pidDriveRight.atSetPoint() && opModeIsActive()) {
            double lPos = lGroup.getPositions().get(0);
            double rPos = rGroup.getPositions().get(0);
            double leftError = pidDriveLeft.calculate(lPos);
            double rightError = pidDriveRight.calculate(rPos);
            double leftSpeed = speed * leftError;
            double rightSpeed = speed * rightError;
            if (!pidDriveLeft.atSetPoint())
                if (leftSpeed > 0) {
                    leftSpeed = Range.clip(leftSpeed, 0.2, 1);
                    leftSpeed *= DRIVE_SPEED_MULTIPLIER;
                    leftSpeed = Range.clip(leftSpeed, 0.2, 1);
                }
                else {
                    leftSpeed = Range.clip(leftSpeed, -1, -0.2);
                    leftSpeed *= DRIVE_SPEED_MULTIPLIER;
                    leftSpeed = Range.clip(leftSpeed, -1, -0.2);
                }
            else
                leftSpeed = 0;

            if (!pidDriveRight.atSetPoint())
                if (rightSpeed > 0) {
                    rightSpeed = Range.clip(rightSpeed, 0.2, 1);
                    rightSpeed *= DRIVE_SPEED_MULTIPLIER;
                    rightSpeed = Range.clip(rightSpeed, 0.2, 1);
                }
                else {
                    rightSpeed = Range.clip(rightSpeed, -1, -0.2);
                    rightSpeed *= DRIVE_SPEED_MULTIPLIER;
                    rightSpeed = Range.clip(rightSpeed, -1, -0.2);
                }
            else
                rightSpeed = 0;

            lGroup.set(leftSpeed);
            rGroup.set(rightSpeed);
            telemetry.addData("Drive PID", "Target: %d, Traveled: %f, %f", targetTicks, lPos, rPos);
            telemetry.addData("Drive PID", "Left Speed: %f, Right Speed: %f", leftError, rightError);
            telemetry.update();
        }
        lGroup.stopMotor();
        rGroup.stopMotor();
        sleep(500);
    }
    private final double INCHES_TO_TICK_MULTIPLIER = 49;// orginal is 49.30

    public int inchesToEncoderTicksInt(double inches) {
        return (int) (inches * INCHES_TO_TICK_MULTIPLIER);
    }

    public void turnPID(double angle, int tolerance) {
        lGroup.resetEncoder();
        rGroup.resetEncoder();

        pidRotate.reset();
        gyro.reset();
        pidRotate.setSetPoint(angle);
        pidRotate.setTolerance(tolerance, 5);
        while (!pidRotate.atSetPoint() && opModeIsActive()) {
            double heading = gyro.getHeading();
            if (heading < -180)
                heading += 360;
            else if (heading > 180)
                heading -= 360;
            double power = pidRotate.calculate(heading);
            if (power > 0)
                power = Range.clip(power, 0.2, 1);
            else
                power = Range.clip(power, -1, -0.2);
            telemetry.addData("Turn PID", "Target angle: %f, Current: %f, Power: %f", angle, gyro.getHeading(), power);
            telemetry.update();
            lGroup.set(-power);
            rGroup.set(power);
            sleep(10);
        }
        lGroup.stopMotor();
        rGroup.stopMotor();
        sleep(500);
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param TurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param angle Absolute Heading Angle (in Degrees) relative to last gyro reset.
     * 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     * If a relative angle is required, add/subtract from current heading.
     */
    public void Turning_Still(double TurnSpeed, double angle) {
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
