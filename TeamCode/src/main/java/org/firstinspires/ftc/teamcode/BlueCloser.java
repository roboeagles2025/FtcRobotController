package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Blue Closer", group = "Autonomous")
@Disabled
public class BlueCloser extends LinearOpMode {
    //private TfodProcessor tfod;
    private VisionPortal visionPortal;
    //Variable that will be used to identify where TeamProp is located:
    //    0 = detected something elsewhere
    //    1 = left
    //    2 = middle
    //    3 = right
    public int detectionVar = 1;

    private final String TFOD_MODEL_FILE = "RoboEagles_Pyramid_640.tflite";
    private final String[] LABELS = { "TeamProp" };
    private final boolean USE_WEBCAM = true;

    // Teamprop Detection Configuration
    // Coordinate for the box associated with Team Prop located in the MIDDLE
    private final double MIDDLE_X_MAX = 700;
    private final double MIDDLE_X_MIN = 100;

    // Coordinate for the box associated with Team Prop located in the RIGHT
    private final double RIGHT_X_MAX = 1200;
    private final double RIGHT_X_MIN = 850;

    private static final double ARMS_SPEED = 0.25;
    public final int WHEEL_TICKS_PER_REV = 560;
    public final int ARM_TICKS_PER_REV = 288;
    public final double WHEEL_DIAMETER = 3.78; // Needs to be changed
    public final double GEAR_RATIO = 1;

    private final double BOTTOM_LEFT_CLAW_OPEN = 0.9;
    private final double BOTTOM_LEFT_CLAW_CLOSE = 0.5;
    private final double BOTTOM_RIGHT_CLAW_CLOSE = 0.5;
    private final double BOTTOM_RIGHT_CLAW_OPEN = 0.1;
    private final double DRIVE_SPEED_MULTIPLIER = 0.5;
    private final double DRIVE_SPEED_MULTIPLIER_MIN = 0.75;
    private final double INCHES_TO_TICK_MULTIPLIER = 49.30;
    private final double IGNORE_Y_MAX = 350;

    //Motor armMotor, elbowMotor;
    Motor elbowMotor;
    MotorGroup lGroup, rGroup;
    MotorEx armMotor;
    Servo blClaw, brClaw;
    PIDController pidDriveLeft, pidDriveRight, pidRotate;
    PIDController pidArm;
    RevIMU gyro;

    @Override
    public void runOpMode() {
        // Get all devices from hardwareMap
        MotorEx flDrive = new MotorEx(hardwareMap, "fl_motor");
        MotorEx frDrive = new MotorEx(hardwareMap, "fr_motor");
        MotorEx blDrive = new MotorEx(hardwareMap, "bl_motor");
        MotorEx brDrive = new MotorEx(hardwareMap, "br_motor");
        blClaw = hardwareMap.servo.get("bottom_left_claw");
        brClaw = hardwareMap.servo.get("bottom_right_claw");
//        armMotor = new Motor(hardwareMap, "arm_motor");
        armMotor = new MotorEx(hardwareMap, "arm_motor");
        elbowMotor = new Motor(hardwareMap, "elbow_motor");
        gyro = new RevIMU(hardwareMap);
        gyro.init();

        blClaw.setPosition(BOTTOM_LEFT_CLAW_CLOSE);
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_CLOSE);

        // Group left motors and right motors together
        lGroup = new MotorGroup(flDrive, blDrive);
        rGroup = new MotorGroup(frDrive, brDrive);

        // Reset all encoders to 0
        lGroup.stopAndResetEncoder();
        rGroup.stopAndResetEncoder();
        armMotor.stopAndResetEncoder();

        // Set left and right motor group to velocity control
        lGroup.setRunMode(Motor.RunMode.RawPower);
        rGroup.setRunMode(Motor.RunMode.RawPower);
        lGroup.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rGroup.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Reverse left motors
        lGroup.setInverted(true);

        pidArm = new PIDController(0.005, 0.0001, 0);
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
        pidDriveLeft = pidDriveRight = new PIDController(0.007, 0.0001, 0);
        pidDriveLeft.setTolerance(30, 20);
        pidDriveRight.setTolerance(30, 20);

        // Initialize the PID controller for turning
        pidRotate = new PIDController(.006, .00008, 0);

        waitForStart();

        // ADD TEAMPROP DETECTION HERE

        int detectionVar = 3;
        switch (detectionVar) {
            case 1:
                spikeMarkLeft();
                break;

            case 2:
                spikeMarkMiddle();
                break;

            case 3:
                spikeMarkRight();
                break;
        }
    }
    private void spikeMarkLeft() {
        driveStraightPID(16);
        turnPID(-55);
        driveStraightPID(5);
        driveStraightPID(-5);
        turnPID(55);
        driveStraightPID(-12);
        turnPID(90);
        driveStraightPID(40);
        blClaw.setPosition(BOTTOM_LEFT_CLAW_OPEN);
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_OPEN);
        sleep(1000);
    }

    private void spikeMarkMiddle() {
        driveStraightPID(22);
        turnPID(-20);
        driveStraightPID(6);
        driveStraightPID(-6);
        turnPID(20);
        driveStraightPID(-18);
        turnPID(90);
        driveStraightPID(40);
        blClaw.setPosition(BOTTOM_LEFT_CLAW_OPEN);
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_OPEN);
        sleep(1000);
    }

    private void spikeMarkRight() {
        driveStraightPID(14);
        turnPID(55);
        driveStraightPID(5);
        driveStraightPID(-5);
        turnPID(-55);
        driveStraightPID(-10);
        turnPID(90);
        driveStraightPID(40);
        blClaw.setPosition(BOTTOM_LEFT_CLAW_OPEN);
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_OPEN);
        sleep(1000);

    }


    private void driveStraightPID(double distance) {
        /*
        I don't think that we need the correction PID because as we are tracking the distance traveled by both the left set of wheels
        and the right set of wheels, if the robot starts to turn, the encoder values will differ and will automatically be corrected
         */
        lGroup.resetEncoder();
        rGroup.resetEncoder();
        pidDriveLeft.reset();
        pidDriveRight.reset();
        int targetTicks = inchesToEncoderTicks(distance);
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
                if (leftSpeed > 0)
                    leftSpeed = Range.clip(leftSpeed, 0.2, 1);
                else
                    leftSpeed = Range.clip(leftSpeed, -1, -0.2);
            else
                leftSpeed = 0;

            if (!pidDriveRight.atSetPoint())
                if (rightSpeed > 0)
                    rightSpeed = Range.clip(rightSpeed, 0.2, 1);
                else
                    rightSpeed = Range.clip(rightSpeed, -1, -0.2);
            else
                rightSpeed = 0;
            // Adjust speed with multiplier
            if (Math.abs(leftSpeed) > DRIVE_SPEED_MULTIPLIER_MIN)
                leftSpeed *= DRIVE_SPEED_MULTIPLIER;
            if (Math.abs(rightSpeed) > DRIVE_SPEED_MULTIPLIER_MIN)
                rightSpeed *= DRIVE_SPEED_MULTIPLIER;

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

    private void turnPID(double angle) {
        lGroup.resetEncoder();
        rGroup.resetEncoder();
        pidRotate.reset();
        gyro.reset();
        pidRotate.setSetPoint(angle);
        pidRotate.setTolerance(3, 5);
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

    private void moveArm(int position) {
        armMotor.set(ARMS_SPEED);
        armMotor.setRunMode(Motor.RunMode.PositionControl);
        armMotor.setTargetPosition(armMotor.getCurrentPosition() + position);
    }

    private void newMoveArm (int ticks) {

        armMotor.resetEncoder();
        pidArm.reset();
        pidArm.setSetPoint(ticks);
        telemetry.addData("moveArm1", "Current position %d, move to: %d", armMotor.getCurrentPosition(), ticks);
        telemetry.update();
        double speed = DRIVE_SPEED_MULTIPLIER;

        while (!pidArm.atSetPoint() && opModeIsActive()) {
            double position = armMotor.getCurrentPosition();

            double armError = pidArm.calculate(position);

            double armSpeed = speed * armError;

            if (!pidArm.atSetPoint())
                if (armSpeed > 0)
                    armSpeed = Range.clip(armSpeed, 0.2, 0.5);
                else
                    armSpeed = Range.clip(armSpeed, -0.5, -0.2);
            else
                armSpeed = 0;

            armMotor.set(armSpeed);
            telemetry.addData("moveArm1 PID", "Target: %d, Traveled: %d", ticks, armMotor.getCurrentPosition());
            telemetry.addData("moveArm1", "error: %f, armSpeed: %f", armError, armSpeed);
            telemetry.update();

        }
        armMotor.set(0);

        sleep(500);



    }

    public double encoderTicksToInches(int ticks) {
        // Found this online from FTC Road Runner
        return WHEEL_DIAMETER * Math.PI * GEAR_RATIO * ticks / WHEEL_TICKS_PER_REV;
    }

    public int inchesToEncoderTicks(double inches) {
//        return (int) ((WHEEL_TICKS_PER_REV * inches) / (Math.PI * WHEEL_DIAMETER * GEAR_RATIO));
        // 943
        //return (int) (inches * 56.7);
        return (int) (inches * INCHES_TO_TICK_MULTIPLIER);
    }
    /**
     * Initializes TFOD and the vision portal
     */
    /*
    public void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();



        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(false);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.90f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }
*/
    /**
     * Uses TFOD to detect team props and returns the side in {@link #detectionVar}
     * If no object is detected, it defaults to left side
     */
    /*
    void detectTeamProp() {
        // Set initially variable to left. Then we will check if something detected in our designated MIDDLE area
        // or RIGHT area and assign variable respectfully. If loop will not be executed (means nothing was detected)
        // it will remain set as LEFT side. But if something was detected but not where we expecting, that it means
        // camera falsly detected something else and we will ignore it with value 0.
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        // Telemetry function to define how many objects are detected.
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            // Check if Spike box is detected/located in the middle
            if (y > IGNORE_Y_MAX) {
                if ((x > MIDDLE_X_MIN) && (x < MIDDLE_X_MAX)) {
                    // Set Detection Variable to middle
                    detectionVar = 2;
                } else {
                    if ((x > RIGHT_X_MIN) && (x < RIGHT_X_MAX)) {
                        // Set Detection Variable to right
                        detectionVar = 3;
                    } else {
                        // haven't detected anything, assume it is LEFT
                        detectionVar = 1;
                    }
                    // We will not change the detection in case multiple boxes were detected and second one is not ours
                }
            }

            telemetry.addLine();
            telemetry.addData("Detection Var", "%d", detectionVar);
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
    }
*/
}




