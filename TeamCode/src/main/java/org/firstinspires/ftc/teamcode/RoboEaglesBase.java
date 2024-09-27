package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class RoboEaglesBase extends LinearOpMode {
    public BNO055IMU imu;
    public DcMotor blDrive, brDrive, flDrive, frDrive;
    public DcMotor elbowMotor;
    public DcMotorEx armMotor;
    public Servo tlClaw, trClaw;
    public Servo blClaw, brClaw;
    public Servo droneLauncher;
    public BNO055IMU.Parameters imuParameters;
    public Orientation robotPosition;
    double globalAngle;
    public Orientation lastAngles = new Orientation();
    public final double ELBOW_SPEED_MULT = 0.75;

    // border values for Bottom Right Claw
    public final double BOTTOM_RIGHT_CLAW_OPEN = 0.5;
    public final double BOTTOM_RIGHT_CLAW_CLOSED = 0.1;
    public final double BOTTOM_RIGHT_CLAW_CENTER = 0.35;
    public final double BOTTOM_RIGHT_CLAW_INIT = BOTTOM_RIGHT_CLAW_CENTER;

    // border values for Bottom Left Claw
    public final double BOTTOM_LEFT_CLAW_OPEN = 0.6;
    public final double BOTTOM_LEFT_CLAW_CLOSED = 1.0;
    public final double BOTTOM_LEFT_CLAW_CENTER = 0.75;
    public final double BOTTOM_LEFT_CLAW_INIT = BOTTOM_LEFT_CLAW_CENTER;
    public final double TOP_LEFT_CLAW_MIN = 0;
    public final double TOP_LEFT_CLAW_MAX = 0.28;
    public final double TOP_LEFT_CLAW_INIT = TOP_LEFT_CLAW_MIN;

    // border values for Top Right Claw
    public final double TOP_RIGHT_CLAW_MIN = 0.25;
    public final double TOP_RIGHT_CLAW_MAX = 0.6;
    public final double TOP_RIGHT_CLAW_INIT = TOP_RIGHT_CLAW_MAX;
    public final double DRONE_LAUNCH_POSITION = 0;
    public final double DRONE_INIT_POSITION = 90;

    // Motor constants
    public final int WHEEL_TICKS_PER_REV = 1120;
    public final int ARM_TICKS_PER_REV = 288;
    public final double WHEEL_DIAMETER = 3.78; // Needs to be changed
    public final double GEAR_RATIO = 1;

    public void mapDevices() {
        // Gyroscope
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Wheels Motor To Code Connection
        brDrive = hardwareMap.dcMotor.get("br_motor");
        blDrive = hardwareMap.dcMotor.get("bl_motor");
        flDrive = hardwareMap.dcMotor.get("fl_motor");
        frDrive = hardwareMap.dcMotor.get("fr_motor");

        // Bottom and top claws - All from the perspective of the robot
        blClaw = hardwareMap.servo.get("bottom_left_claw");
        brClaw = hardwareMap.servo.get("bottom_right_claw");
        tlClaw = hardwareMap.servo.get("left_finger");
        trClaw = hardwareMap.servo.get("right_finger");

        droneLauncher = hardwareMap.servo.get("drone_servo");

        elbowMotor = hardwareMap.dcMotor.get("elbow_motor");
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");

        // Wheel configuration
        // LEFT Side forward
        blDrive.setDirection(DcMotor.Direction.REVERSE);
        flDrive.setDirection(DcMotor.Direction.REVERSE);

        // RIGHT Side Reversed
        brDrive.setDirection(DcMotor.Direction.FORWARD);
        frDrive.setDirection(DcMotor.Direction.FORWARD);

        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Arm + Elbow motor configuration
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetMotors();

        // Bottom claw configuration

        // Drone launcher configuration
        droneLauncher.setPosition(DRONE_INIT_POSITION);

        // IMU (gyroscope) configuration
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = true;
        resetIMU();
        robotPosition = imu.getAngularOrientation();
    }

    void resetMotors() {
        frDrive.setMode(RunMode.STOP_AND_RESET_ENCODER);
        flDrive.setMode(RunMode.STOP_AND_RESET_ENCODER);
        brDrive.setMode(RunMode.STOP_AND_RESET_ENCODER);
        blDrive.setMode(RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);

        idle();
        // Motors are in RUN_WITHOUT_ENCODER by default, but if in autonomous,
        // they will be switched into RUN_TO_POSITION
        frDrive.setMode(RunMode.RUN_WITHOUT_ENCODER);
        flDrive.setMode(RunMode.RUN_WITHOUT_ENCODER);
        brDrive.setMode(RunMode.RUN_WITHOUT_ENCODER);
        blDrive.setMode(RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
    }

    void moveSidesPosition(int ticks, double power) {
        flDrive.setTargetPosition(flDrive.getCurrentPosition() + ticks);
        frDrive.setTargetPosition(frDrive.getCurrentPosition() + ticks);
        blDrive.setTargetPosition(blDrive.getCurrentPosition() + ticks);
        brDrive.setTargetPosition(brDrive.getCurrentPosition() + ticks);

        flDrive.setMode(RunMode.RUN_TO_POSITION);
        frDrive.setMode(RunMode.RUN_TO_POSITION);
        blDrive.setMode(RunMode.RUN_TO_POSITION);
        brDrive.setMode(RunMode.RUN_TO_POSITION);

        flDrive.setPower(power);
        frDrive.setPower(power);
        blDrive.setPower(power);
        brDrive.setPower(power);
    }

    /**
     * Simple method to set all motors with same power for certain period of time
     * Note: the motors will NOT stop at the end
     *
     * @param timeToMove time in milliseconds for how long to move the motors.
     * @param speed      what power to set ALL motors to (range -1.0 to +1.0).
     */
    void moveAllWheels(long timeToMove, double speed) {
        flDrive.setPower(speed);
        blDrive.setPower(speed);
        frDrive.setPower(speed);
        brDrive.setPower(speed);
        sleep(timeToMove);
    }

    /**
     * Method to set left and right side motors with different power for certain period of time
     * Note: the motors will NOT stop at the end
     *
     * @param timeToMove time in milliseconds for how long to move the motors.
     * @param speedLeft  what power to set LEFT side motors to (range -1.0 to +1.0).
     * @param speedRight what power to set RIGHT side motors to (range -1.0 to +1.0).
     */
    void moveSidesSame(long timeToMove, double speedLeft, double speedRight) {
        flDrive.setPower(speedLeft);
        blDrive.setPower(speedLeft);
        frDrive.setPower(speedRight);
        brDrive.setPower(speedRight);
        sleep(timeToMove);
    }

    /**
     * Method to set left and right side motors with different power for certain period of time
     * Note: the motors will NOT stop at the end
     *
     * @param timeToMove time in milliseconds for how long to move the motors.
     * @param speedFlBr  what power to set Front Left and Back Right motors to (range -1.0 to +1.0).
     * @param speedFrBl  what power to set Front Right and Back Left motors to (range -1.0 to +1.0).
     */
    void moveSidesDiagonal(long timeToMove, double speedFlBr, double speedFrBl) {

        frDrive.setPower(speedFrBl);
        brDrive.setPower(speedFlBr);
        flDrive.setPower(speedFlBr);
        blDrive.setPower(speedFrBl);

        sleep(timeToMove);
    }

    public double getRobotPosition() {
        //robotPosition = imu.getAngularOrientation();
        //return robotPosition.firstAngle;
        return 0;

    }


    public void resetIMU() {
        //imu.initialize(imuParameters);
        sleep(500);
    }


    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    public double getImuAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetImuAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public double encoderTicksToInches(int ticks) {
        // Found this online from FTC Road Runner
        return WHEEL_DIAMETER * Math.PI * GEAR_RATIO * ticks / WHEEL_TICKS_PER_REV;
    }

    public double inchesToEncoderTicks(double inches) {
        return (WHEEL_TICKS_PER_REV * inches) / (Math.PI * WHEEL_DIAMETER);

    }
}