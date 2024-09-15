package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "FTCLib TeleOp", group = "Concept")
public class TeleOpNew extends LinearOpMode {
    private final double CLAW_INCREMENT = 0.02;
    public final double BOTTOM_RIGHT_CLAW_CENTER = 0.35;
    public final double BOTTOM_RIGHT_CLAW_INIT = BOTTOM_RIGHT_CLAW_CENTER;

    // border values for Bottom Left Claw
    public final double BOTTOM_LEFT_CLAW_CENTER = 0.75;
    public final double BOTTOM_LEFT_CLAW_INIT = BOTTOM_LEFT_CLAW_CENTER;
    public final double TOP_LEFT_CLAW_MIN = 0;
    public final double TOP_LEFT_CLAW_MAX = 0.28;
    public final double TOP_LEFT_CLAW_INIT = TOP_LEFT_CLAW_MIN;

    // border values for Top Right Claw
    public final double TOP_RIGHT_CLAW_MIN = 0.25;
    public final double TOP_RIGHT_CLAW_MAX = 0.6;
    public final double TOP_RIGHT_CLAW_INIT = TOP_RIGHT_CLAW_MAX;

    private final double BOTTOM_LEFT_CLAW_MAX = 0.9;
    private final double BOTTOM_LEFT_CLAW_MIN = 0.5;
    private final double BOTTOM_RIGHT_CLAW_MAX = 0.5;
    private final double BOTTOM_RIGHT_CLAW_MIN = 0.1;

    MecanumDrive drive;
    Motor armMotor, elbowMotor;
    Servo blClaw, brClaw;
    Servo tlClaw, trClaw;
    Servo droneLauncher;
    private double topLeftClawPosition = TOP_LEFT_CLAW_INIT;
    private double topRightClawPosition = TOP_RIGHT_CLAW_INIT;
    private double bottomLeftClawPosition = BOTTOM_LEFT_CLAW_INIT;
    private double bottomRightClawPosition = BOTTOM_RIGHT_CLAW_INIT;

    @Override
    public void runOpMode() {
        MotorEx flDrive = new MotorEx(hardwareMap, "fl_motor");
        MotorEx frDrive = new MotorEx(hardwareMap, "fr_motor");
        MotorEx blDrive = new MotorEx(hardwareMap, "bl_motor");
        MotorEx brDrive = new MotorEx(hardwareMap, "br_motor");
        armMotor = new Motor(hardwareMap, "arm_motor");
        elbowMotor = new Motor(hardwareMap, "elbow_motor");
        blClaw = hardwareMap.servo.get("bottom_left_claw");
        brClaw = hardwareMap.servo.get("bottom_right_claw");
        tlClaw = hardwareMap.servo.get("left_finger");
        trClaw = hardwareMap.servo.get("right_finger");
        droneLauncher = hardwareMap.servo.get("drone_servo");
        drive = new MecanumDrive(flDrive, frDrive, blDrive, brDrive);
        elbowMotor.setRunMode(Motor.RunMode.RawPower);

        flDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        frDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        blDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        brDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        brClaw.setPosition(BOTTOM_RIGHT_CLAW_INIT);
        blClaw.setPosition(BOTTOM_LEFT_CLAW_INIT);
        droneLauncher.setPosition(90);
        waitForStart();
        while (opModeIsActive()) {
            checkDriving();
            checkBottomClaw();
            checkTopClaw();
            checkArm();
            checkElbow();
            checkDroneLaunch();
            telemetry.update();
            sleep(10);
        }
    }

    private void checkDriving() {
        drive.driveRobotCentric(gamepad1.left_stick_x * 0.75, gamepad1.right_stick_y * 0.75, -gamepad1.right_stick_x * 0.75);
    }

    private void checkArm() {
        armMotor.setRunMode(Motor.RunMode.VelocityControl);
        armMotor.set(gamepad2.left_stick_y / 2); // Increase divisor more less sensitivity and decrease for more sensitivity
    }

    private void checkElbow() {
        // Elbow doesn't really need to be accurate, so we can use raw power
        elbowMotor.set(gamepad2.right_stick_y);
    }

    private void checkDroneLaunch() {
        if (gamepad2.right_bumper)
            droneLauncher.setPosition(0);
    }
    void checkTopClaw() {
        boolean close_servo = gamepad2.a;    // Close fingers
        boolean open_servo = gamepad2.b;     // open fingers
        telemetry.addData("TopClaw", "Open: %b, Close: %b", open_servo, close_servo);
        // slew the servo
        if (close_servo) {
            // Left finger will go to maximum value and then stay at that position
            topLeftClawPosition += CLAW_INCREMENT;
            if (topLeftClawPosition >= TOP_LEFT_CLAW_MAX) {
                topLeftClawPosition = TOP_LEFT_CLAW_MAX;
            }
            // Right finger will go to minimum value and then stay at that position
            topRightClawPosition -= CLAW_INCREMENT;
            if (topRightClawPosition <= TOP_RIGHT_CLAW_MIN) {
                topRightClawPosition = TOP_RIGHT_CLAW_MIN;
            }
        }

        if (open_servo) {
            // Left finger will go to minimum value and then stay at that position
            topLeftClawPosition -= CLAW_INCREMENT ;
            if (topLeftClawPosition <= TOP_LEFT_CLAW_MIN ) {
                topLeftClawPosition = TOP_LEFT_CLAW_MIN;
            }
            // Right finger will go to maximum value and then stay at that position
            topRightClawPosition += CLAW_INCREMENT ;
            if (topRightClawPosition >= TOP_RIGHT_CLAW_MAX ) {
                topRightClawPosition = TOP_RIGHT_CLAW_MAX;
            }
        }

        // Move both servos to new position.  Assume servos are mirror image of each other.
        telemetry.addData("TopClaw", "Left Claw: %f, Right Claw: %f", topLeftClawPosition, topRightClawPosition);
        tlClaw.setPosition(topLeftClawPosition);
        trClaw.setPosition(topRightClawPosition);
    }

    void checkBottomClaw() {
        boolean close_servo = gamepad2.x;    // Close fingers
        boolean open_servo = gamepad2.y;     // open fingers
        telemetry.addData("BottomClaw", "Open: %b, Close: %b", open_servo, close_servo);
        // slew the servo
        if (open_servo) {
            // Left finger will go to maximum value and then stay at that position
            bottomLeftClawPosition += CLAW_INCREMENT;
            if (bottomLeftClawPosition >= BOTTOM_LEFT_CLAW_MAX) {
                bottomLeftClawPosition = BOTTOM_LEFT_CLAW_MAX;
            }
            // Right finger will go to minimum value and then stay at that position
            bottomRightClawPosition -= CLAW_INCREMENT;
            if (bottomRightClawPosition <= BOTTOM_RIGHT_CLAW_MIN) {
                bottomRightClawPosition = BOTTOM_RIGHT_CLAW_MIN;
            }
        }

        if (close_servo) {
            // Left finger will go to minimum value and then stay at that position
            bottomLeftClawPosition -= CLAW_INCREMENT;
            if (bottomLeftClawPosition <= BOTTOM_LEFT_CLAW_MIN) {
                bottomLeftClawPosition = BOTTOM_LEFT_CLAW_MIN;
            }
            // Right finger will go to maximum value and then stay at that position
            bottomRightClawPosition += CLAW_INCREMENT;
            if (bottomRightClawPosition >= BOTTOM_RIGHT_CLAW_MAX) {
                bottomRightClawPosition = BOTTOM_RIGHT_CLAW_MAX;
            }
        }
        telemetry.addData("BottomClaw", "Left Claw: %f, Right Claw: %f", bottomLeftClawPosition, bottomRightClawPosition);
        // Move both servos to new position.  Assume servos are mirror image of each other.
        blClaw.setPosition(bottomLeftClawPosition);
        brClaw.setPosition(bottomRightClawPosition);
    }
}
