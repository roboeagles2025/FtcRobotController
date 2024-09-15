package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


@SuppressWarnings("FieldCanBeLocal") // To get rid of the annoying "variable can be made local" warnings
public abstract class RoboEaglesAutonomousBaseNew extends RoboEaglesBaseNew {


    // Teamprop Detection Configuration
    // Coordinate for the box associated with Team Prop located in the MIDDLE
    private final double MIDDLE_X_MAX = 700;
    private final double MIDDLE_X_MIN = 100;

    // Coordinate for the box associated with Team Prop located in the RIGHT
    private final double RIGHT_X_MAX = 1200;
    private final double RIGHT_X_MIN = 850;

    private final double IGNORE_Y_MAX = 350;

    private final String TFOD_MODEL_FILE = "RoboEagles_Pyramid_640.tflite";
    private final String[] LABELS = { "TeamProp" };
    private final boolean USE_WEBCAM = true;

    // Global variables
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private int aprilTagID = 0;


    //Variable that will be used to identify where TeamProp is located:
    //    0 = detected something elsewhere
    //    1 = left
    //    2 = middle
    //    3 = right
    public int detectionVar = 1;

    // Set PID proportional value to start reducing power at about 50 degrees of rotation.
    // P by itself may stall before turn completed so we add a bit of I (integral) which
    // causes the PID controller to gently increase power if the turn is not completed.
    PIDController pidRotate = new PIDController(.003, .00003, 0);

    // Set PID proportional value to produce non-zero correction value when robot veers off
    // straight line. P value controls how sensitive the correction is.
    PIDController pidDrive = new PIDController(.05, 0, 0);

    double correction;

    // Subclasses will set this
    protected int backboardInches;
    protected String teamColor; // "BLUE" or "RED"

    abstract void spikeMarkLeft();
    abstract void spikeMarkMiddle();
    abstract void spikeMarkRight();

    private boolean ENABLE_TFOD = true;
    public void autonomousStart() {

        // Initialize hardware devices
        mapDevices();

        if (ENABLE_TFOD) {
            initTfod();
            // 1. Initialize Camera and TensorFlow
            // we disabling AprilTag for now as we are not using it
            // aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
            /*
             * The INIT-loop:
             * This REPLACES waitForStart!
             */

            // Enable the TFOD processor for our TeamProp Detection.
            while (!isStarted() && !isStopRequested()) {
                detectTeamProp();
                // Do all the other stuff
                telemetry.update();
                sleep(10);
            }
        }
        else {
            detectionVar = 2;
            waitForStart();
        }

        blClaw.setPosition(BOTTOM_LEFT_CLAW_CLOSE + 0.05);
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_CLOSE - 0.05);

        switch (detectionVar) {
            case 1: // Left Side
                spikeMarkLeft();
                break;

            case 2: // Middle
                spikeMarkMiddle();
                break;

            case 3: // Right Side
                spikeMarkRight();
        }
    }


    /**
     * Initializes TFOD and the vision portal
     */
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

    /**
     * Function to add telemetry about AprilTag detections.
     */
    private int detectAprilTags() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        int count = currentDetections.size();
        telemetry.addData("# AprilTags Detected", count);

        if (count == 1) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    aprilTagID = detection.id;
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }

            }   // end for() loop
        }

        return count;

    }

    /**
     * Uses TFOD to detect team props and returns the side in {@link #detectionVar}
     * If no object is detected, it defaults to left side
     */
    public void detectTeamProp() {
        // Set initially variable to left. Then we will check if something detected in our designated MIDDLE area
        // or RIGHT area and assign variable respectfully. If loop will not be executed (means nothing was detected)
        // it will remain set as LEFT side. But if something was detected but not where we expecting, that it means
        // camera falsly detected something else and we will ignore it with value 0.
        List<Recognition> currentRecognitions = tfod.getRecognitions();

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

    /**
     * Function to turn the robot using PID controller
     * @param angle Angle to turn. Positive is left, Negative is right.
     */
    public void turnPID(double angle) { turnPID(angle, 3); }

    public void turnPID(double angle, int tolerance) {
        lGroup.resetEncoder();
        rGroup.resetEncoder();
//        lGroup.setRunMode(Motor.RunMode.VelocityControl);
//        rGroup.setRunMode(Motor.RunMode.VelocityControl);
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

    public void moveArm(int position) {
        armMotor.set(ARMS_SPEED);
        armMotor.setRunMode(Motor.RunMode.PositionControl);
        armMotor.setTargetPosition(armMotor.getCurrentPosition() + position);
    }

    public void newMoveArm (int ticks) {

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


}
