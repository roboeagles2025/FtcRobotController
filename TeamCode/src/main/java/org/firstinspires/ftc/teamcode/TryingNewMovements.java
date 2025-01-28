package org.firstinspires.ftc.teamcode;// Import FTC SDK classes
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
//package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.RoboEaglesAutoBase2425;

    @Autonomous(name = "MovementTest", group = "Autonomous")
    public class TryingNewMovements extends RoboEaglesAutoBase2425 {
    // Hardware declarations
    /*private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;*/

    // Constants for PID control
    private static final double kP = 0.05;  // Proportional gain
    private static final double kI = 0.01;  // Integral gain
    private static final double kD = 0.02;  // Derivative gain

    // PID control variables
    private double previousError = 0;
    private double integral = 0;
    private double currentTime = 0;
    private double maxPower = 0.8;  // Max motor power
    private double minPower = 0.1;  // Min motor power

    // Time-based movement (in seconds)
    private double targetTime;  // The target time to move

    private ElapsedTime runtime = new ElapsedTime();  // Timer for runtime

        //Override
    public void inittest() {
        // Initialize hardware
        /*flDriveEx = new MotorEx(hardwareMap, "fl_motor");
        frDriveEx = new MotorEx(hardwareMap, "fr_motor");
        blDriveEx = new MotorEx(hardwareMap, "bl_motor");
        brDriveEx = new MotorEx(hardwareMap, "br_motor");*/

        // Reset the runtime timer
        runtime.reset();
    }
    public void runOpMode() {
        MapDevicesTesting();
        inittest();
        while (!isStarted() && !isStopRequested()) {
            //   detectTeamProp();
            // Do all the other stuff
            //telemetry.update();
            sleep(10);
        }
        looptest();
        sleep(100);
    }
    //@Override
    public void looptest() {
        // Example usage of each movement function (you can call these based on your input)

        // Move forward 24 inches
            /*moveForward(24);
            moveBackward(24);*/
           // turnRight(90);
            turnRight(30);



        // Update telemetry
        telemetry.addData("Runtime", runtime.seconds());
        telemetry.update();
    }

    // Helper method to set motor powers
    private void setMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        flDriveEx.set(frontLeftPower);
        frDriveEx.set(frontRightPower);
        blDriveEx.set(backLeftPower);
        brDriveEx.set(backRightPower);
    }

    // Reset PID control variables and timer
    private void resetPID() {
        previousError = 0;
        integral = 0;
        runtime.reset();
    }

    // Move forward using PID control based on time
    public void moveForward(double targetDistance) {
        resetPID();
        targetTime = targetDistance / 23;  // Example: 12 inches per second (adjust for your robot)

        // Loop to control movement using PID
        while (opModeIsActive() && runtime.seconds() < targetTime) {
            double error = targetTime - runtime.seconds();

            // Calculate PID terms
            double pTerm = kP * error;
            integral += error;
            double iTerm = kI * integral;
            double dTerm = kD * (error - previousError);
            previousError = error;

            // Calculate motor power
            double power = pTerm + iTerm + dTerm;
            power = Math.max(minPower, Math.min(maxPower, power));  // Limit motor power

            // Set motors to move forward
            setMotorPowers(power, power, power, power);
        }
        stopMotors();
    }

    // Move backward using PID control based on time
    public void moveBackward(double targetDistance) {
        resetPID();
        targetTime = targetDistance / 23;  // Example: 12 inches per second

        // Loop to control movement using PID
        while (opModeIsActive() && runtime.seconds() < targetTime) {
            double error = targetTime - runtime.seconds();

            // Calculate PID terms
            double pTerm = kP * error;
            integral += error;
            double iTerm = kI * integral;
            double dTerm = kD * (error - previousError);
            previousError = error;

            // Calculate motor power
            double power = pTerm + iTerm + dTerm;
            power = Math.max(minPower, Math.min(maxPower, power));  // Limit motor power

            // Set motors to move backward
            setMotorPowers(-power, -power, -power, -power);
        }
        stopMotors();
    }

    // Turn left using PID control (using time-based movement)
    private void turnLeft(double targetAngle) {
        resetPID();
        targetTime = targetAngle / 100;  // Example: 90 degrees per second

        // Loop to control turning using PID
        while (opModeIsActive() && runtime.seconds() < targetTime) {
            double error = targetTime - runtime.seconds();

            // Calculate PID terms
            double pTerm = kP * error;
            integral += error;
            double iTerm = kI * integral;
            double dTerm = kD * (error - previousError);
            previousError = error;

            // Calculate motor power
            double power = pTerm + iTerm + dTerm;
            power = Math.max(minPower, Math.min(maxPower, power));  // Limit motor power

            // Set motors to turn left
            setMotorPowers(-power, power, -power, power);
        }
        stopMotors();
    }

    // Turn right using PID control (using time-based movement)
    private void turnRight(double targetAngle) {
        resetPID();
        targetTime = targetAngle / 90;  // Example: 90 degrees per second
        //targetTime = 2;
        // Loop to control turning using PID
        while (opModeIsActive() && runtime.seconds() < targetTime) {
            double error = targetTime - runtime.seconds();

            // Calculate PID terms
            double pTerm = kP * error;
            integral += error;
            double iTerm = kI * integral;
            double dTerm = kD * (error - previousError);
            previousError = error;

            // Calculate motor power
            double power = pTerm + iTerm + dTerm;
            power = Math.max(minPower, Math.min(maxPower, power));  // Limit motor power

            // Set motors to turn right
            setMotorPowers(power, -power, power, -power);
        }
        stopMotors();
    }

    // Strafe left using PID control
    private void strafeLeft(double targetDistance) {
        resetPID();
        targetTime = targetDistance / 12.0;  // Example: 12 inches per second

        // Loop to control strafing using PID
        while (opModeIsActive() && runtime.seconds() < targetTime) {
            double error = targetTime - runtime.seconds();

            // Calculate PID terms
            double pTerm = kP * error;
            integral += error;
            double iTerm = kI * integral;
            double dTerm = kD * (error - previousError);
            previousError = error;

            // Calculate motor power
            double power = pTerm + iTerm + dTerm;
            power = Math.max(minPower, Math.min(maxPower, power));  // Limit motor power

            // Set motors to strafe left
            setMotorPowers(-power, power, power, -power);
        }
        stopMotors();
    }

    // Strafe right using PID control
    private void strafeRight(double targetDistance) {
        resetPID();
        targetTime = targetDistance / 12.0;  // Example: 12 inches per second

        // Loop to control strafing using PID
        while (opModeIsActive() && runtime.seconds() < targetTime) {
            double error = targetTime - runtime.seconds();

            // Calculate PID terms
            double pTerm = kP * error;
            integral += error;
            double iTerm = kI * integral;
            double dTerm = kD * (error - previousError);
            previousError = error;

            // Calculate motor power
            double power = pTerm + iTerm + dTerm;
            power = Math.max(minPower, Math.min(maxPower, power));  // Limit motor power

            // Set motors to strafe right
            setMotorPowers(power, -power, -power, power);
        }
        stopMotors();
    }

    // Helper method to stop all motors
    private void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }
}
