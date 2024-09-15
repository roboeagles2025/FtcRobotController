package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Objects;

@SuppressWarnings("FieldCanBeLocal") // To get rid of the annoying "variable can be made local" warnings
public abstract class RoboEaglesAutonomousBase extends RoboEaglesBase {
    // Settings for robot on the field
    // MAKE SURE TO CHECK THESE SETTINGS IN COMPETITION
    // BACKSTAGE_PARKING defines whether to park in backstage
    // MAKE SURE TO DISABLE WHEN ON FARTHER SIDE
    public final boolean BACKSTAGE_PARKING = true;
    // If BACKSTAGE_PARKING is true, BACKSTAGE_PARKING_POSITION defines the position to park in backstage (from POV of robot)
    // 1 - Next to the side walls
    // 2 - In front of backdrop
    // 3 - Next to the backdrop in the middle of field
    public final int BACKSTAGE_PARKING_POSITION = 1;

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

    // Wheels configuration
    private final int ONE_SECOND = 1000;
    private final double INCHES_PER_SECOND = 56;
    private final double LEFT_SIDE_MULTIPLIER    = 1;
    private final double RIGHT_SIDE_MULTIPLIER   = 1;
    private final double TURN_MAX_POWER = 0.5;
    private final double DRIVE_MAX_POWER = 0.75;
    private final int ZERO_POWER = 0;
    private double leftPower, rightPower;
    private double leftPowerD, rightPowerD;
    private final String LEFT_DIRECTION = "Left";
    private final double     SMOOTH_RATIO            = 0.125 ;   // how much time (%) use for smooth start/stop
    private final int        ITERATION_MIN           = 5 ;
    private final int CYCLE_MS = 50;
    private final double     INCHES_PER_SECOND_SIDEWAYS       = 7.5 ;    // this is currently working to drive 10 inches
    private final double     DEGREE_PER_SEC          = 380 ;   // 0.5 speed with reverse sides

    public final boolean EXTEND = true;
    public final boolean RETRACT = false;

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

    abstract void placePurplePixelLeft();
    abstract void placePurplePixelMiddle();
    abstract void placePurplePixelRight();

    public void autonomousStart() {
        mapDevices();

        // 1. Initialize Camera and TensorFlow
        initTfod();
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
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_CLOSED);
        blClaw.setPosition(BOTTOM_LEFT_CLAW_CLOSED);

        switch (detectionVar) {
            case 1: // Left Side
                placePurplePixelLeft();
                break;

            case 2: // Middle
                placePurplePixelMiddle();
                break;

            case 3: // Right Side
                placePurplePixelRight();
        }
        backstageParking(detectionVar);
    }

    /**
     * Parks the robot in the backstage.
     * Moves the robot according to {@link #BACKSTAGE_PARKING} and {@link #BACKSTAGE_PARKING_POSITION}.
     */
    private void backstageParking(int spikeMarkSide) {
        if (BACKSTAGE_PARKING) {
            if (backboardInches > 60) // Temporarily disable backstage parking
                return;
            if ((Objects.equals(teamColor, "RED") && spikeMarkSide == 3) || (Objects.equals(teamColor, "BLUE") && spikeMarkSide == 1))
                return;
            // Move into backstage
            driveStraightSmoothly(0.5, backboardInches);
            sleep(1000);

            // HACK: Since blue and red are basically the same just mirrored,
            // direction will be 1 or -1 and will be multiplied to every turn
            int dir = Objects.equals(teamColor, "RED") ? 1 : -1;
            if (BACKSTAGE_PARKING_POSITION == 1)
                backstageParkingCenter(dir);
            else if (BACKSTAGE_PARKING_POSITION == 3)
                backstageParkingWall(dir);
        }
    }

    /**
     * Parks the robot in the center of the field, next to the backdrop
     * @param dir 1 for Red alliance, -1 for Blue alliance
     */
    private void backstageParkingCenter(int dir) {
        // THIS IS JUST A DRAFT
        turnPID(dir * -90, 0.7);
        sleep(1000);

        driveStraightSmoothly(0.5, 15);
        sleep(1000);


        turnPID(dir * 90, 0.7);
        sleep(1000);

        tlClaw.setPosition(TOP_LEFT_CLAW_MAX);
        trClaw.setPosition(TOP_RIGHT_CLAW_MIN);
        sleep(1000);
//        driveStraightSmoothly(0.75, -5);
    }

    /**
     * Parks the robot between the side wall and the backdrop
     * @param dir 1 for Red alliance, -1 for Blue alliance
     */
    private void backstageParkingWall(int dir) {
        turnPID(dir * 90, 0.7);
        sleep(1000);

        driveStraightSmoothly(0.75, 20);
        sleep(1000);

        turnPID(dir * 90, 0.7);
        sleep(1000);

        driveStraightSmoothly(0.75, -10);
    }

    /**
     * Moves robot to the board on the blue side
     * @param inches Number of inches to the board
     * @deprecated {@link #backstageParking(int)} will now take care of everything
     */
    @Deprecated
    public void moveToBoardBlue(double inches) {
        driveStraightSmoothly(0.75, inches);
        sleep(1000);
        // turn right 90 degree toward to the middle of the field
        turnPID(-85, 0.7);
        sleep(1000);
        // move forward to avoid backboard
        driveStraightSmoothly(0.75, 20);
        sleep(1000);
        // turn left 90 degree to face the wall again
        turnPID(90, 0.7);
        sleep(1000);
        // move forward a bit to make sure robot is in parking space
        driveStraightSmoothly(0.75, 10);
    }

    /**
     * Moves robot to the board on the red side
     * @param inches Number of inches to the board
     * @deprecated {@link #backstageParking(int)} will now take care of everything
     */
    @Deprecated
    void moveToBoardRed(double inches) {
        driveStraightSmoothly(0.75, inches);
        sleep(1000);
        // turn left 90 degree toward to the middle of the field
        turnPID(90, 0.7);
        sleep(1000);
        // move forward to avoid backboard
        driveStraightSmoothly(0.75, 20);
        sleep(1000);
        // turn right 90 degree to face the wall again
        turnPID(-90, 0.7);
        sleep(1000);
        // move forward a bit to make sure robot is in parking space
        driveStraightSmoothly(0.75, 10);
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
    void detectTeamProp() {
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

    /**
     *  Method to set rotate robot to a target angle using PID method
     *  Note: the motors WILL stop at the end
     *
     * @param targetAngle   Angle (in Degrees) relative to last gyro reset.
     *                      Positive value is turning to the left (CounterClockWise).
     *                      Negative value is turning to the right (ClockWise).
     * @param initialKp     the value of KP to be used in the calculations
     */
    void turnPID(double targetAngle, double initialKp) {
        double power, prevError, error, dT, prevTime, curTime;
        error = targetAngle - getRobotPosition();
        curTime = 0;
        double kP = initialKp; // INITIALIZE THESE LATER
        double kI = 0.0;
        double kD = 0;
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
                power = Range.clip(power, 0.23, 1);
            else
                power = Range.clip(power, -1, -0.23);
            flDrive.setPower(-power);
            blDrive.setPower(-power);
            frDrive.setPower(power);
            brDrive.setPower(power);
            telemetry.addData("TurnPID", "Motor powers: %f", power);
            telemetry.addData("TurnPID", "Error: %f", error);
            telemetry.addData("TurnPID", "Yaw position: %f", robotPosition.firstAngle);
            telemetry.update();
            //sleep(10);
        }
        moveAllWheels(10,0);
        resetIMU();
    }

    void turnPIDNew(int degrees) {
        resetImuAngle();

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setOutputRange(0, TURN_MAX_POWER); // Output Range is [-0.5, 0.5]
        // Maximum input will be 110% of target angle to handle overshooting
        // Input Range will be reversed if negative degrees
        pidRotate.setInputRange(0, 1.1 * degrees); // Must change minimum input because motors don't move the robot under ~10% power
        pidRotate.setTolerance(1); // Will declare "on target" if within 1% of target angle
        pidRotate.enable();
        double power;
        do {
            power = pidRotate.performPID(getImuAngle());
            moveSidesSame(0, -power, power);
        } while (!pidRotate.onTarget() && opModeIsActive());
        moveAllWheels(0, 0);
        pidRotate.disable();
    }

    public void driveStraightPid(double driveSpeed, double distance) {
        resetImuAngle();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, 0.75);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
        double absDistance = Math.abs (distance);
        long totalTime = (long) Math.round((absDistance * ONE_SECOND) / (INCHES_PER_SECOND * driveSpeed * LEFT_SIDE_MULTIPLIER));

        // drive for certain amount of time (CYCLE_MS) before calculating new correction.
        while (totalTime > 0) {
            correction = pidDrive.performPID(getImuAngle());

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            //telemetry.addData("4 turn rotation", rotation);

            moveSidesSame(CYCLE_MS, driveSpeed + correction, driveSpeed - correction);
            totalTime = totalTime - CYCLE_MS;
        }
        pidDrive.disable();

        moveAllWheels(0,0);
    }

    public void driveStraightPIDNew(double distance) {
        resetImuAngle();
        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, 0.25); // Output range is [-0.25, 0.25]
        pidDrive.setInputRange(0, 90); // Input range is [-90, 90]
        pidDrive.setTolerance(1); // Considered "on target" if within 1% of degree
        pidDrive.enable();

        moveSidesPosition((int) inchesToEncoderTicks(distance), 0.75);
        while (frDrive.isBusy() && flDrive.isBusy() && brDrive.isBusy() && blDrive.isBusy()) {
            // Even though we are setting the power to 0.75,
            // I **think** that in RUN_TO_POSITION, as the motor nears its destination,
            // It will automatically slow down and won't actually go at 75% power
            if (!pidDrive.onTarget()) {
                double correction = pidDrive.performPID(getImuAngle());
                moveSidesSame(0, DRIVE_MAX_POWER - correction, DRIVE_MAX_POWER + correction);
            }
                moveAllWheels(0, DRIVE_MAX_POWER);
        }
        moveAllWheels(0, 0);
        pidDrive.disable();
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

            if (Objects.equals(direction, LEFT_DIRECTION)){
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

    void moveArmAndStay(double Angle, double speed)
    {
        long timeToMove = (long) Math.abs(Math.round(Angle)) * 5;
        double armPower = speed;
        if (Angle < 0)
        {
            armPower = -speed;
        }
        armMotor.setPower(armPower);
        sleep(timeToMove);
        armMotor.setPower(0.25);
    }

    void moveArm(double angle) {
        int ticks = (int) (ARM_TICKS_PER_REV * (angle / 360));
        // Lets say that we want it to move in one second
        // Ticks per second will be simply be ticks
        armMotor.setTargetPosition(ticks);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(ticks);
    }

    void stopArmMovement()
    {
        armMotor.setPower(0);
    }

    void moveElbow(boolean extend) {
        double power = 1;
        if (!extend)
            power *= -1;
        elbowMotor.setPower(power * ELBOW_SPEED_MULT);
        sleep(2500);
        elbowMotor.setPower(0);
    }

    void liftArm(double speed) {
        armMotor.setPower(speed);
    }

    void placePixelBoard() {
        liftArm(0.25);
        sleep(1000);
        moveElbow(EXTEND);
        sleep(500);
        driveStraightSmoothly(0.25, 10);
        sleep(500);
        liftArm(-0.15);
        sleep(1000);
        liftArm(0.05);
        tlClaw.setPosition(TOP_LEFT_CLAW_MAX);
        trClaw.setPosition(TOP_RIGHT_CLAW_MIN);
        sleep(500);
        driveStraightSmoothly(0.25, -10);
        sleep(500);
        moveElbow(RETRACT);
        sleep(500);
        liftArm(0);
    }
}
