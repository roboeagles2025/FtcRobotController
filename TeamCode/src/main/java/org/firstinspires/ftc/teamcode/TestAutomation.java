package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.bosch.BNO055IMU;
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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Test Automation", group = "Autonomous")
@Disabled
public class TestAutomation extends LinearOpMode {

    // Coordinate for the box associated with Team Prop located in the MIDDLE
    private static final double MIDDLE_X_MAX = 750;
    private static final double MIDDLE_X_MIN = 250;
    private static final double MIDDLE_Y_MAX = 500;
    private static final double MIDDLE_Y_MIN = 370;

    // Coordinate for the box associated with Team Prop located in the RIGHT
    private static final double RIGHT_X_MAX = 1200;
    private static final double RIGHT_X_MIN = 925;
    private static final double RIGHT_Y_MIN = 420;
    private static final double RIGHT_Y_MAX = 600;


    // Coordinates to ignore even if detected something
    private static final double IGNORE_Y_MAX = 365;
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    private int AprilTagID = 0;
    private TfodProcessor tfod;

    private static final String TFOD_MODEL_FILE = "model_20231028_130313.tflite";
//    private static final String TFOD_MODEL_FILE = "RoboEagles_TeamProp_320.tflite";
    private static final String TFOD_MODEL_ASSET = "CenterStage.tflite";
    private static final String[] LABELS = {
            "TeamProp",
    };

    private static final boolean USE_WEBCAM = true;
    private final int ONE_SECOND = 1000;
    private final double INCHES_PER_SECOND = 56;
    private final double     INCHES_PER_SECOND_SIDEWAYS       = 7.5 ;    // this is currently working to drive 10 inches
    private final double     DEGREE_PER_SEC          = 380 ;   // 0.5 speed with reverse sides
    private final double     LEFT_SIDE_MULTIPLIER    = 1 ;
    private final double     RIGHT_SIDE_MULTIPLIER   = 1 ;
    private final double     SMOOTH_RATIO            = 0.125 ;   // how much time (%) use for smooth start/stop
    private final int        ITERATION_MIN           = 5 ;
    private final String LEFT_DIRECTION = "Left";
    private final String RIGHT_DIRECTION = "Right";
    private final int ZERO_POWER = 0;
    static final int CYCLE_MS = 50;
    private VisionPortal visionPortal;

    //Variable that will be used to identify where TeamProp is located:
    //    0 = detected something elsewhere
    //    1 = left
    //    2 = middle
    //    3 = right
    private int detectionVar = 0;
    private BNO055IMU imu;
    private double leftPower, rightPower;
    private double leftPowerD, rightPowerD;
    private DcMotor blDrive, brDrive, flDrive, frDrive;
    private DcMotor rightArm, leftArm;
    private Servo tlClaw, trClaw;
    private Servo blClaw, brClaw;
    private DcMotor tcConnector;
    private AprilTagProcessor aprilTagProcessor;

    // border values for Bottom Right Claw
    private final double BOTTOM_RIGHT_CLAW_MAX = 0.5;
    private final double BOTTOM_RIGHT_CLAW_MIN = 0.1;
    private final double BOTTOM_RIGHT_CLAW_INIT = BOTTOM_RIGHT_CLAW_MAX;

    // border values for Bottom Left Claw
    private final double BOTTOM_LEFT_CLAW_MAX = 1.0;
    private final double BOTTOM_LEFT_CLAW_MIN = 0.6;
    private final double BOTTOM_LEFT_CLAW_INIT = BOTTOM_LEFT_CLAW_MIN;

    Orientation robotPosition;
    BNO055IMU.Parameters imuParameters;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        brDrive = hardwareMap.get(DcMotor.class, "br_motor");
        blDrive = hardwareMap.get(DcMotor.class, "bl_motor");
        flDrive = hardwareMap.get(DcMotor.class, "fl_motor");
        frDrive = hardwareMap.get(DcMotor.class, "fr_motor");

//        rightArm = hardwareMap.get(DcMotor.class, "right_arm");
//        leftArm = hardwareMap.get(DcMotor.class, "left_arm");

//        tlClaw = hardwareMap.get(Servo.class, "left_finger");
//        trClaw = hardwareMap.get(Servo.class, "right_finger");

        blClaw = hardwareMap.get(Servo.class, "bottom_left_claw");
        brClaw = hardwareMap.get(Servo.class, "bottom_right_claw");

//        tcConnector = hardwareMap.get(DcMotor.class, "connect_arm");

        // Initalizes the drives rotation direction.
        // LEFT Side forward
        blDrive.setDirection(DcMotor.Direction.REVERSE);
        flDrive.setDirection(DcMotor.Direction.REVERSE);
        // RIGHT Side Reversed
        brDrive.setDirection(DcMotor.Direction.FORWARD);
        frDrive.setDirection(DcMotor.Direction.FORWARD);

//        tcConnector.setDirection(DcMotor.Direction.REVERSE);
//        tcConnector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        rightArm.setDirection(DcMotor.Direction.FORWARD);
//        leftArm.setDirection(DcMotor.Direction.REVERSE);

//        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine(String.format("Bottom Claw Right: %f\n", brClaw.getPosition()));
        telemetry.addLine(String.format("Bottom Claw Left: %f\n", blClaw.getPosition()));
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_INIT);
        blClaw.setPosition(BOTTOM_LEFT_CLAW_INIT);

        imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = true;
        resetIMU();
        robotPosition = imu.getAngularOrientation();

        // 1. Initialize Camera and TensorFlow
        initTfod();
        /*
        // we disabling AprilTag for now as we are not using it


        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        */
        // 2. Report if Spike is detected (or reported that nothing detected)
//        detectionVar = detectTeamProp();
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        // Enable the TFOD processor for our TeamProp Detection.
        visionPortal.setProcessorEnabled(tfod, true);
        while (!isStarted() && !isStopRequested()) {

            if (detectionVar == 0) {
                visionPortal.setProcessorEnabled(tfod, true);
                visionPortal.setProcessorEnabled(aprilTag, false);
                detectionVar = detectTeamProp();
                // Do all the other stuff
                sleep(10);
            } else {
                visionPortal.setProcessorEnabled(tfod, false);
                visionPortal.setProcessorEnabled(aprilTag, true);
                detectAprilTags();
                sleep(10);
            }
            telemetry.update();
        }
        // Disable the TFOD processor since we have done TeamProp Detection,
        // from now on we need AprilTag, so lets enable AprilTag
        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(aprilTag, true);
        turnPID(90);

        sleep(1000);

        turnPID(-90);
        sleep(1000);
        turnPID(90);
        turnPID(90);
        sleep(1000);
//
//        //while (detectAprilTags() != 1);
//        detectAprilTags();
//        visionPortal.setProcessorEnabled(aprilTag, false);
//
//        //waitForStart();
//
//        // LEFT ???
//        if (detectionVar == 1) {
//            placePurplePixelLeft();
//            // For left and middle, move forward 82 inches
//            MoveToTheBoard(82);
//        }
//        // MIDDLE ???
//        if (detectionVar == 2) {
//            placePurplePixelMiddle();
//            // For middle and middle, move forward 82 inches
//            MoveToTheBoard(82);
//        }
//
//        // RIGHT ????
//        if (detectionVar == 3) {
//            // Add moving to the one squre to its right
//
//            placePurplePixelRight();
//            // For right, move forward 58.5 inches
//            // MoveToTheBoard(58);
//        }
//

        // For right, move forward 58.5 inches
        // Create function to grab the pixel from the bottom claw into the top claw
        // Place the pixel on backboard if we can, or place in backstage
        // Park the robot



    }

    private void MoveToTheBoard(double inches) {
        driveStraightSmoothly(0.5, inches);
        sleep(1000);
        /*
        Turning_Still(0.75, -90);
        sleep(1000);
        driveStraightSmoothly(0.75, 15);
        sleep(1000);
        Turning_Still(0.75, 90);
        sleep(1000);
        driveStraightSmoothly(0.75, 10);

         */
    }
    private void placePurplePixelLeft() {
        // Move 23.5 inches forward
        driveStraightSmoothly(0.5, 25);
        sleep(2000);
        // Rotate 90 degrees
        Turning_Still(0.5, -75);
        sleep(2000);
        // Move few inches forward
        driveStraightSmoothly(0.5, 3);
        sleep(2000);
        // Open claw
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_MIN);
        blClaw.setPosition(BOTTOM_LEFT_CLAW_MAX);
        // Move few inches backward
        driveStraightSmoothly(0.5, -7);
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_MAX);
        blClaw.setPosition(BOTTOM_LEFT_CLAW_MIN);
        sleep(2000);
        // Rotate 180 degrees
        Turning_Still(0.5, 130);
        sleep(2000);
    }

    private void placePurplePixelMiddle() {
        // Move 23.5 inches forward
        driveStraightSmoothly(0.5, 31);
        sleep(2000);
        // Rotate 90 degrees
        //Turning_Still(0.5, -20);
        //sleep(2000);
        // Move few inches forward
        //driveStraightSmoothly(0.5, 3);
        //sleep(2000);
        // Open claw
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_MIN);
        blClaw.setPosition(BOTTOM_LEFT_CLAW_MAX);
        sleep(1000);
        // Move few inches backward
        driveStraightSmoothly(0.5, -4);
        sleep(2000);
        // Rotate 180 degrees
        //Turning_Still(0.5, 20);
        //sleep(2000);
        Turning_Still(0.5, 80);
        sleep(2000);
    }

    private void placePurplePixelRight() {

        // Move 23.5 inches forward
        driveStraightSmoothly(0.5, 23.5);
        sleep(2000);
        // Rotate 90 degrees
        Turning_Still(0.5, 80);
        sleep(2000);
        // Move few inches forward
        driveStraightSmoothly(0.5, 5);
        sleep(2000);
        // Open claw
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_MIN);
        blClaw.setPosition(BOTTOM_LEFT_CLAW_MAX);
        sleep(1000);
        // Move few inches backward
        driveStraightSmoothly(0.5, -3);
        sleep(2000);

/*
        // move forward 3 inches
        driveStraightSmoothly(0.5, 3);
        sleep(2000);
        // rotate 90 degrees to the right (Positive)
        Turning_Still(0.5, 90);
        sleep(2000);
        // move forward 23.5 inches
        driveStraightSmoothly(0.5, 23.5);
        sleep(2000);
        // rotate 90 degree to the left
        Turning_Still(0.5, -90);
        sleep(2000);
        // move backward slowly 5 inches
        driveStraightSmoothly(0.3, -5);
        sleep(2000);
        // run same way as we did left side
        placePurplePixelLeft();


 */
    }

    private void parkInBackStage() {

       driveStraightSmoothly(0.75, 82.5);
    }





    //more functions
    private void initTfod() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

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

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

        // Make sure to disable the aprilTag processor since we will be using tfod first.
        visionPortal.setProcessorEnabled(aprilTag, false);
    }   // end method initTfod()


    /**
     * Function to add telemetry about AprilTag detections.
     */
    private int detectAprilTags() {
        int count = 0;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        count = currentDetections.size();
        telemetry.addData("# AprilTags Detected", count);

        if (count == 1) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    AprilTagID = detection.id;
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

    }   // end method telemetryAprilTag()

    int detectTeamProp() {
        // Set initially variable to left. Then we will check if something detected in our designated MIDDLE area
        // or RIGHT area and assign variable respectfully. If loop will not be executed (means nothing was detected)
        // it will remain set as LEFT side. But if something was detected but not where we expecting, that it means
        // camera falsly detected something else and we will ignore it with value 0.
        int retVal = 1;
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            // Check if Spike box is detected/located in the middle
            if ((x > MIDDLE_X_MIN) && (x < MIDDLE_X_MAX) && (y > MIDDLE_Y_MIN) && (y < MIDDLE_Y_MAX)) {
                    // Set Detection Variable to middle
                retVal = 2;
            } else {
                if ((x > RIGHT_X_MIN) && (x < RIGHT_X_MAX) && (y > RIGHT_Y_MIN) && (y < RIGHT_Y_MAX)) {
                    // Set Detection Variable to right
                    retVal = 3;
                } else if (y < IGNORE_Y_MAX){
                    // detecting something else, just ignoring it
                } else
                {
                    // haven't detected anything, assume it is LEFT
                    retVal = 1;
                }
                // We will not change the detection in case multiple boxes were detected and second one is not ours
            }

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.addData("Detection_Var", "%d", retVal);

        }   // end for() loop
        return retVal;
    }

    /**
     * Method to move to the position indicated by spike mark or team prop
     * @param targetAprilTag Position indicated by spike mark or team prop (1 for left, 2 for middle, 3 for right)
     */
    private void moveToAprilTag(int targetAprilTag) {
        // The April Tag values are 4, 5, 6 so just add 3 to get target num
        targetAprilTag += 3;
        boolean targetAprilTagDetected = false;
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == targetAprilTag) {
                targetAprilTagDetected = true;
                // Move sideways (angle is detection.ftcPose.bearing
                break;
            }
        }
        if (!targetAprilTagDetected) {
            if (targetAprilTag > detections.get(0).id) {
                // April Tag is to the right
                // Move sideways (angle is arbitrary value, maybe 60 degrees)
            } else {
                // April Tag is to the left
                // Move sideways (-60 degrees)
            }

        }
    }



    /**
     *  Method to drive in a straight line, at certain speed until it covers the desired distance
     *
     * @param driveSpeed Speed for motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param direction  Determine which direction to move sideways (values: "Left" or "Right")
     */
    public void driveSideways(double driveSpeed,
                              double distance,
                              String direction) {

        //double leftMultiplier = 0.50;
        //double rightMultiplier = 0.50;
        double absDistance = Math.abs (distance);

        // distance should be divided by how many inches robot covers per second
        // then adjust by robot's speed
        //long timeToDrive = (long) (((driveSpeed * Math.abs(distance)) / INCHES_PER_SECOND ) * ONE_SECOND);
        long timeToDrive = (long) Math.round((absDistance * ONE_SECOND) / (INCHES_PER_SECOND * driveSpeed * LEFT_SIDE_MULTIPLIER));

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            if (direction == LEFT_DIRECTION){
                // direction to the RIGHT is considered "forward". If chosen left motors needs to be reversed
                leftPowerD = driveSpeed * LEFT_SIDE_MULTIPLIER * -1.0;
                rightPowerD = driveSpeed * RIGHT_SIDE_MULTIPLIER;
            } else {
                leftPowerD = driveSpeed * LEFT_SIDE_MULTIPLIER;
                rightPowerD = driveSpeed * RIGHT_SIDE_MULTIPLIER * -1.0;
            }

            // Set diagonal motors
            flDrive.setPower(leftPowerD);
            blDrive.setPower(rightPowerD);
            brDrive.setPower(leftPowerD);
            frDrive.setPower(rightPowerD);

            sleep(timeToDrive);
            flDrive.setPower(ZERO_POWER);
            frDrive.setPower(ZERO_POWER);
            blDrive.setPower(ZERO_POWER);
            brDrive.setPower(ZERO_POWER);
            telemetry.addData("Motors", "left (%.2f), right (%.2f), time to drive (%d)", leftPowerD, rightPowerD, timeToDrive);
        }
    }

    /**
     *  Method to drive in a straight line, at certain speed until it covers the desired distance
     *  This method intends to smooth start and stop
     *
     * @param driveSpeed Speed for motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     */
    public void driveStraightSmoothly (double driveSpeed, double distance){
        double absDistance = Math.abs (distance);						// Absolute value of the distance that will be using for all the calculations
        double altDriveSpeedLeft = driveSpeed * LEFT_SIDE_MULTIPLIER;	// value of the speed for LEFT side motors adjusted by respective modifier
        double altDriveSpeedRight = driveSpeed * RIGHT_SIDE_MULTIPLIER;	// value of the speed for RIGHT side motors adjusted by respective modifier

        // multiplier to reverse movement. Must be either 1.0 or -1.0 if distance is negative
        double speedDirection = 1.0;
        if (distance < 0){
            speedDirection = -1.0;
        }

        // calculate total time that needed to cover the distance. For the simplicity we will use LEFT side modifiers in this calculations
        long totalTime = (long) Math.round((absDistance * ONE_SECOND) / (INCHES_PER_SECOND * driveSpeed * LEFT_SIDE_MULTIPLIER));

        //long totalTimeLeft = (long) Math.round((absDistance * ONE_SECOND) / (INCHES_PER_SECOND * driveSpeed * LEFT_SIDE_MULTIPLIER));
        //long totalTimeRight = (long) Math.round((absDistance * ONE_SECOND) / (INCHES_PER_SECOND * driveSpeed * RIGHT_SIDE_MULTIPLIER));
        //long totalTime = Math.max(totalTimeLeft,totalTimeRight);

        long smoothPhaseTime = (long) (totalTime * SMOOTH_RATIO);		// what time should be taking for accelerating to or decelerating from full speed
        long countDown = smoothPhaseTime;								// countdown for the loop. Starts from maximum time and will go down to 0 in the loop
        double distanceCovered = 0;										// track the distance covered while accelerating
        double distanceFullSpeed = 0.0;									// track the distance covered at full speed

        long timeFullSpeed = totalTime;									// value of the time that needed to cover remaining distance at full speed
        //long timeFullSpeedLeft = totalTime;			 					// value of the time that needed to cover remaining distance at full speed for LEFT motors
        //long timeFullSpeedRight = totalTime;							// value of the time that needed to cover remaining distance at full speed for RIGHT motors

        // calculate number of iterations that will be used for smooth start. Minimum is 5
        // Start with 50ms for duration of one iteration, but change if it is give us less than 5 iterations
        long timeIteration = CYCLE_MS;
        int numberIterations = Math.round(smoothPhaseTime / timeIteration);
        if (numberIterations < ITERATION_MIN){
            numberIterations = ITERATION_MIN;
            timeIteration = smoothPhaseTime / numberIterations;
        }

        // Calculate speed adjustment per each iteration for each side motors
        double speedIterationLeft =  altDriveSpeedLeft / numberIterations;		// LEFT side motors
        double speedIterationRight =  altDriveSpeedRight / numberIterations;	// RIGHT side motors
        double speedRatioLeft = 0;										// value of speed that will be slowly increased or decreased for LEFT side. Start with 0.
        double speedRatioRight = 0;										// value of speed that will be slowly increased or decreased for RIGHT side. Start with 0.

        //************************
        // smoothly accelerate
        //************************
        while (countDown > 0) {
            speedRatioLeft = speedRatioLeft + speedIterationLeft;
            speedRatioRight = speedRatioRight + speedIterationRight;
            moveSidesSame(timeIteration, (speedRatioLeft * speedDirection), (speedRatioRight * speedDirection));
            distanceCovered = distanceCovered + ((speedRatioLeft * timeIteration * INCHES_PER_SECOND) / ONE_SECOND);
            countDown = countDown - timeIteration;
        }

        telemetry.addData("SmoothAccelerate", "Accelerate time = (%d), Distance covered = (%.2f)", smoothPhaseTime, distanceCovered );

        //************************
        // recalculate time for remaining distance and run at the full speed
        //************************
        distanceFullSpeed = absDistance - (distanceCovered * 2);
        timeFullSpeed = (long) Math.round((distanceFullSpeed * ONE_SECOND) / (INCHES_PER_SECOND * altDriveSpeedLeft)) ;
        moveSidesSame(timeFullSpeed, (altDriveSpeedLeft * speedDirection), (altDriveSpeedRight * speedDirection));

        telemetry.addData("SmoothFullSpeed", "Full speed time = (%d), Distance covered = (%.2f)", timeFullSpeed, distanceFullSpeed);
        telemetry.addData("SmoothData", "Total time = (%d), Left speed = (%.2f), Right speed = (%.2f)", (timeFullSpeed + (smoothPhaseTime * 2)), altDriveSpeedLeft, altDriveSpeedRight);

        //************************
        // smoothly decelerate
        //************************
        double distanceCovered2 = 0;										// track the distance covered while decelerating
        speedRatioLeft = altDriveSpeedLeft;
        speedRatioRight = altDriveSpeedRight;
        countDown = smoothPhaseTime;
        while (countDown > 0) {
            moveSidesSame(timeIteration, (speedRatioLeft * speedDirection), (speedRatioRight * speedDirection));
            distanceCovered2 = distanceCovered2 + ((speedRatioLeft * timeIteration * INCHES_PER_SECOND) / ONE_SECOND);
            speedRatioLeft = speedRatioLeft - speedIterationLeft;
            speedRatioRight = speedRatioRight - speedIterationRight;
            countDown = countDown - timeIteration;
        }

        //************************
        //completely stop
        //************************
        moveAllWheels(0,0);
    }


    /**
     *  Method to drive in a straight line, at certain speed until it covers the desired distance
     *  This method intends to smooth start and stop
     *
     * @param driveSpeed Speed for motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param direction  Determine which direction to move sideways (values: "Left" or "Right")
     */
    public void driveSidewaySmoothly (double driveSpeed, double distance, String direction){
        double absDistance = Math.abs (distance);						// Absolute value of the distance that will be using for all the calculations
        double altDriveSpeedLeft = driveSpeed * LEFT_SIDE_MULTIPLIER;	// value of the speed for LEFT side motors adjusted by respective modifier
        double altDriveSpeedRight = driveSpeed * RIGHT_SIDE_MULTIPLIER;	// value of the speed for RIGHT side motors adjusted by respective modifier
        double leftSideForward = 1.0;
        double rightSideForward = 1.0;

        if (direction == LEFT_DIRECTION){
            // direction to the RIGHT is considered "forward". If chosen left motors needs to be reversed
            leftSideForward = -1.0;
            rightSideForward = 1.0;
        } else {
            leftSideForward = 1.0;
            rightSideForward = -1.0;
        }

        // multiplier to reverse movement. Must be either 1.0 or -1.0 if distance is negative
        double speedDirection = 1.0;
        if (distance < 0){
            speedDirection = -1.0;
        }



        // calculate total time that needed to cover the distance. For the simplicity we will use LEFT side modifiers in this calculations
        long totalTime = (long) Math.round((absDistance * ONE_SECOND) / (INCHES_PER_SECOND_SIDEWAYS * driveSpeed * LEFT_SIDE_MULTIPLIER));

        long smoothPhaseTime = (long) (totalTime * SMOOTH_RATIO);		// what time should be taking for accelerating to or decelerating from full speed
        long countDown = smoothPhaseTime;								// countdown for the loop. Starts from maximum time and will go down to 0 in the loop
        double distanceCovered = 0;										// track the distance covered while accelerating
        double distanceFullSpeed = 0.0;									// track the distance covered at full speed

        long timeFullSpeed = totalTime;									// value of the time that needed to cover remaining distance at full speed

        // calculate number of iterations that will be used for smooth start. Minimum is 5
        // Start with 50ms for duration of one iteration, but change if it is give us less than 5 iterations
        long timeIteration = CYCLE_MS;
        int numberIterations = Math.round(smoothPhaseTime / timeIteration);
        if (numberIterations < ITERATION_MIN){
            numberIterations = ITERATION_MIN;
            timeIteration = smoothPhaseTime / numberIterations;
        }

        // Calculate speed adjustment per each iteration for each side motors
        double speedIterationLeft =  altDriveSpeedLeft / numberIterations;		// LEFT side motors
        double speedIterationRight =  altDriveSpeedRight / numberIterations;	// RIGHT side motors
        double speedRatioLeft = 0;										// value of speed that will be slowly increased or decreased for LEFT side. Start with 0.
        double speedRatioRight = 0;										// value of speed that will be slowly increased or decreased for RIGHT side. Start with 0.

        //************************
        // smoothly accelerate
        //************************
        while (countDown > 0) {
            speedRatioLeft = speedRatioLeft + speedIterationLeft;
            speedRatioRight = speedRatioRight + speedIterationRight;

            moveSidesDiagonal(timeIteration, (speedRatioLeft * speedDirection * leftSideForward), (speedRatioRight * speedDirection * rightSideForward));

            distanceCovered = distanceCovered + ((speedRatioLeft * timeIteration * INCHES_PER_SECOND_SIDEWAYS) / ONE_SECOND);
            countDown = countDown - timeIteration;
        }

        telemetry.addData("SmoothAccelerate", "Accelerate time = (%d), Distance covered = (%.2f)", smoothPhaseTime, distanceCovered );

        //************************
        // recalculate time for remaining distance and run at the full speed
        //************************
        distanceFullSpeed = absDistance - (distanceCovered * 2);
        timeFullSpeed = (long) Math.round((distanceFullSpeed * ONE_SECOND) / (INCHES_PER_SECOND_SIDEWAYS * altDriveSpeedLeft)) ;
        moveSidesDiagonal(timeIteration, (altDriveSpeedLeft * speedDirection * leftSideForward), (altDriveSpeedRight * speedDirection * rightSideForward));

        telemetry.addData("SmoothFullSpeed", "Full speed time = (%d), Distance covered = (%.2f)", timeFullSpeed, distanceFullSpeed);
        telemetry.addData("SmoothData", "Total time = (%d), Left speed = (%.2f), Right speed = (%.2f)", (timeFullSpeed + (smoothPhaseTime * 2)), altDriveSpeedLeft, altDriveSpeedRight);

        //************************
        // smoothly decelerate
        //************************
        double distanceCovered2 = 0;										// track the distance covered while decelerating
        speedRatioLeft = altDriveSpeedLeft;
        speedRatioRight = altDriveSpeedRight;
        countDown = smoothPhaseTime;
        while (countDown > 0) {
            moveSidesDiagonal(timeIteration, (speedRatioLeft * speedDirection * leftSideForward), (speedRatioRight * speedDirection * rightSideForward));
            distanceCovered2 = distanceCovered2 + ((speedRatioLeft * timeIteration * INCHES_PER_SECOND_SIDEWAYS) / ONE_SECOND);
            speedRatioLeft = speedRatioLeft - speedIterationLeft;
            speedRatioRight = speedRatioRight - speedIterationRight;
            countDown = countDown - timeIteration;
        }

        //************************
        //completely stop
        //************************
        moveAllWheels(0,0);
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


    /**
     *  Simple method to open or close claw. It does not have ability to be in middle position
     *
     * @param open    TRUE:  if we want to open the claw
     *                FALSE: if we want to close the claw
     */
/*    public void Move_Claw(boolean open) {


        // slew the servo
        if (open) {
            // Left finger will go to maximum value and then stay at that position
            positionLF = MAX_POS_LF ;

            // Right finger will go to minimum value and then stay at that position
            positionRF = MIN_POS_RF ;

        } else {
            // Left finger will go to minimum value and then stay at that position
            positionLF = MIN_POS_LF ;

            // Right finger will go to maximum value and then stay at that position
            positionRF = MAX_POS_RF ;

        }

        // Display the current value
        telemetry.addData("Claw Position", "LF = %5.2f, RF = %5.2f ", positionLF, positionRF);
        //telemetry.update();

        // Set the servo to the new position and pause;
        clawRFServo.setPosition(positionRF);
        clawLFServo.setPosition(positionLF);
        idle();
    }*/

    /**
     *  Simple method to set all motors with same power for certain period of time
     *  Note: the motors will NOT stop at the end
     *
     * @param timeToMove    time in milliseconds for how long to move the motors.
     * @param speed         what power to set ALL motors to (range -1.0 to +1.0).
     */
    void moveAllWheels (long timeToMove, double speed) {

        flDrive.setPower(speed);
        blDrive.setPower(speed);
        frDrive.setPower(speed);
        brDrive.setPower(speed);
        sleep(timeToMove);
    }

    /**
     *  Method to set left and right side motors with different power for certain period of time
     *  Note: the motors will NOT stop at the end
     *
     * @param timeToMove    time in milliseconds for how long to move the motors.
     * @param speedLeft     what power to set LEFT side motors to (range -1.0 to +1.0).
     * @param speedRight    what power to set RIGHT side motors to (range -1.0 to +1.0).
     */
    void moveSidesSame (long timeToMove, double speedLeft, double speedRight) {
        flDrive.setPower(speedLeft);
        blDrive.setPower(speedLeft);
        frDrive.setPower(speedRight);
        brDrive.setPower(speedRight);
        sleep(timeToMove);
    }

    /**
     *  Method to set left and right side motors with different power for certain period of time
     *  Note: the motors will NOT stop at the end
     *
     * @param timeToMove    time in milliseconds for how long to move the motors.
     * @param speedFlBr     what power to set Front Left and Back Right motors to (range -1.0 to +1.0).
     * @param speedFrBl     what power to set Front Right and Back Left motors to (range -1.0 to +1.0).
     */
    void moveSidesDiagonal (long timeToMove, double speedFlBr, double speedFrBl) {

        frDrive.setPower(speedFrBl);
        brDrive.setPower(speedFlBr);
        flDrive.setPower(speedFlBr);
        blDrive.setPower(speedFrBl);

        sleep(timeToMove);
    }

    void turnPID(double targetAngle) {
        double power, prevError, error, dT, prevTime, curTime;
        error = targetAngle - getRobotPosition();
        curTime = prevError = 0;
        double kP = 0.7; // INITIALIZE THESE LATER
        double kI = 0.0;
        double kD = 0;
        power = kP;
        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive()) {
            prevError = error;
            error = targetAngle - getRobotPosition();
            prevTime = curTime;
            curTime = time.milliseconds();
            dT = curTime - prevTime; //GET DIFFERENCE IN CURRENT TIME FROM PREVIOUS TIME
            power = (error * kP) + ((error) * dT * kI) + (((error - prevError)/dT) * kD);
            power /= 100;
            if (power < 0.05 && power > -0.05)
                break;
            if (power > 0)
                power = Range.clip(power, 0.13, 1);
            else
                power = Range.clip(power, -1, -0.13);
            flDrive.setPower(-power);
            blDrive.setPower(-power);
            frDrive.setPower(power);
            brDrive.setPower(power);
            telemetry.addLine(String.format("Motor powers: %f", power));
            telemetry.addLine(String.format("Error: %f", error));
            telemetry.addLine(String.format("Yaw position: %f", robotPosition.firstAngle));
            telemetry.update();
            //sleep(10);
        }
        moveAllWheels(10,0);
        resetIMU();
    }

    double getRobotPosition() {
        robotPosition = imu.getAngularOrientation();
        return robotPosition.firstAngle;
    }

    void resetIMU() {
        imu.initialize(imuParameters);
        sleep(500);
    }
}
