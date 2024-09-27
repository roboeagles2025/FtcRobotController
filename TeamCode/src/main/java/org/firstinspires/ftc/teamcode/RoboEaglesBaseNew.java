package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;



public abstract class RoboEaglesBaseNew extends LinearOpMode {

    public static final double ARMS_SPEED = 0.25;
    public final int WHEEL_TICKS_PER_REV = 560;
    public final int ARM_TICKS_PER_REV = 288;
    public final double WHEEL_DIAMETER = 3.78; // Needs to be changed
    public final double GEAR_RATIO = 1;

    public final double BOTTOM_LEFT_CLAW_OPEN = 0.9;
    public final double BOTTOM_LEFT_CLAW_CLOSE = 0.5;
    public final double BOTTOM_RIGHT_CLAW_CLOSE = 0.5;
    public final double BOTTOM_RIGHT_CLAW_OPEN = 0.1;
    public final double DRIVE_SPEED_MULTIPLIER = 0.5;
    public final double DRIVE_SPEED_MULTIPLIER_MIN = 0.75;

    Motor elbowMotor;
    MotorGroup lGroup, rGroup;
    MotorEx armMotor;
    Servo blClaw, brClaw;
    public PIDController pidDriveLeft, pidDriveRight, pidRotate;
    PIDController pidArm;
    RevIMU gyro;



    public void mapDevices() {
        // Get all devices from hardwareMap
        MotorEx flDrive = new MotorEx(hardwareMap, "fl_motor");
        MotorEx frDrive = new MotorEx(hardwareMap, "fr_motor");
        MotorEx blDrive = new MotorEx(hardwareMap, "bl_motor");
        MotorEx brDrive = new MotorEx(hardwareMap, "br_motor");
        blClaw = hardwareMap.servo.get("bottom_left_claw");
        brClaw = hardwareMap.servo.get("bottom_right_claw");
        //armMotor = new Motor(hardwareMap, "arm_motor");
        armMotor = new MotorEx(hardwareMap, "arm_motor");
        elbowMotor = new Motor(hardwareMap, "elbow_motor");
        gyro = new RevIMU(hardwareMap);
        gyro.init();

        blClaw.setPosition(BOTTOM_LEFT_CLAW_CLOSE + 0.05);
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_CLOSE - 0.05);

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

        pidArm = new com.arcrobotics.ftclib.controller.PIDController(0.005, 0.0001, 0);
        pidArm.setTolerance(10, 10);


        // Initialize the PID controllers for the left motor group and right motor group
        // P - Proportional - Figured out through trial and error
        // I - Integral - "Impatience" - Will increase speed if error hasn't changed over time
        // D - Derivative - Will counteract proportional and integral - Disabled for now
        pidDriveLeft = pidDriveRight = new com.arcrobotics.ftclib.controller.PIDController(0.005, 0.0001, 0);
        pidDriveLeft.setTolerance(30, 20);
        pidDriveRight.setTolerance(30, 20);

        // Initialize the PID controller for turning
        pidRotate = new PIDController(.006, .00008, 0);


    }

    public double encoderTicksToInches(int ticks) {
        // Found this online from FTC Road Runner
        return WHEEL_DIAMETER * Math.PI * GEAR_RATIO * ticks / WHEEL_TICKS_PER_REV;
    }
    private final double INCHES_TO_TICK_MULTIPLIER = 49.30;


    public int inchesToEncoderTicksInt(double inches) {
        return (int) (inches * INCHES_TO_TICK_MULTIPLIER);
    }

}
