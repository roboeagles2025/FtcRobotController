package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import java.util.List;
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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@TeleOp(name="Robo09/04", group="OpMode")
public class TeleopGamepadTesting extends RoboEaglesBase {


    private double MOTOR_SPEED_MULT = 0.7;
    void MapDevicesTesting() {
        brDrive = hardwareMap.dcMotor.get("br_motor");
        blDrive = hardwareMap.dcMotor.get("bl_motor");
        flDrive = hardwareMap.dcMotor.get("fl_motor");
        frDrive = hardwareMap.dcMotor.get("fr_motor");
        brClaw = hardwareMap.servo.get("br_claw");
        blClaw = hardwareMap.servo.get("bl_claw");
    }
    private final double CLAW_INCREMENT = 0.02;
    private final double BOTTOM_RIGHT_CLAW_MAX = 0.5;
    private final double BOTTOM_RIGHT_CLAW_MIN = 0;
    private final double BOTTOM_RIGHT_CLAW_INIT = BOTTOM_RIGHT_CLAW_MIN;
    private final double BOTTOM_LEFT_CLAW_MAX = 1.0;
    private final double BOTTOM_LEFT_CLAW_MIN = 0.5;
    private final double BOTTOM_LEFT_CLAW_INIT = BOTTOM_LEFT_CLAW_MAX;
    private double bottomLeftClawPosition = BOTTOM_LEFT_CLAW_INIT;
    private double bottomRightClawPosition = BOTTOM_RIGHT_CLAW_INIT;
    private double ARM_SPEED_MULT = 0.5;
    private double MOTOR_SPEED_MULT = 0.7;

    public void runOpMode() {
        MapDevicesTesting();

        waitForStart();
        while (opModeIsActive()) {
            checkDriving();
            checkBottomClaw();
            telemetry.update();
            sleep(10);
        }
    }



    void checkDriving() {
        if (gamepad1.left_stick_x != 0 || gamepad1.right_stick_y != 0)
            checkDrivingNormal();
        else if (gamepad1.right_stick_x != 0 || gamepad1.left_stick_y != 0)
            checkDrivingStrafing();
            //checkDrivingNormal();
        else {
            flDrive.setPower(0);
            frDrive.setPower(0);
            blDrive.setPower(0);
            brDrive.setPower(0);
        }
    }


    void checkDrivingNormal() {
        double drive = gamepad1.left_stick_y;
        double turn = -gamepad1.left_stick_x;
        //double turn = 0;
       double leftPower = Range.clip(drive + turn, -1, 1);
       double rightPower = Range.clip(drive - turn, -1, 1);

        //double leftPower = Range.clip(drive + turn, -1, 1);
        //double rightPower = -leftPower;

        leftPower *= MOTOR_SPEED_MULT;
        rightPower *= MOTOR_SPEED_MULT;


        telemetry.addData("2026 DrivingNormal", "Joystick Drive: %f, Turn: %f", drive, turn);
        telemetry.addData("DrivingNormal", "Left Power: %f, Right Power: %f", leftPower, rightPower);

        moveSidesSame(0, leftPower, -1*(rightPower));

        //flDrive.setPower(leftPower);
        //blDrive.setPower(leftPower);
        //frDrive.setPower(rightPower*(-1));
        //brDrive.setPower(rightPower*(-1));
    }


    void checkDrivingStrafing() {
        double drive = gamepad1.right_stick_y;
        double turn = gamepad1.right_stick_x;
        if (drive == 0 && turn == 0) return;

        double flBrPower = Range.clip(drive + turn, -1, 1);
        double frBlPower = Range.clip(drive - turn, -1, 1);


        flBrPower *= MOTOR_SPEED_MULT;
        frBlPower *= MOTOR_SPEED_MULT;

        telemetry.addData("DrivingStrafing", "Joystick Drive: %f, Turn: %f", drive, turn);
        telemetry.addData("DrivingStrafing", "FlBr Power: %f, FrBl Power: %f", flBrPower, frBlPower);

        moveSidesDiagonal(0, flBrPower, frBlPower);
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
        //private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
        void checkArm() {
            double power = gamepad2.left_stick_y;
//        armMotor.setTargetPosition(armMotor.getTargetPosition() + (int) (power  * 10));
//        telemetry.addData("Arm", "Target Position: %d", armMotor.getTargetPosition());
            power *= ARM_SPEED_MULT;
            armMotor.setTargetPosition(armMotor.getTargetPosition() + (int) (power * 5));
            armMotor.setPower(power);
        }










            // Step through the list of detections and display info for each one.



    }   // end class
}



