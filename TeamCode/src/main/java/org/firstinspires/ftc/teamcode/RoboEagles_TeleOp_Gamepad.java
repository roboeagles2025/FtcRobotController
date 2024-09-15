package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

@TeleOp(name="RoboEagles TeleOp Gamepad.", group="OpMode")
public class RoboEagles_TeleOp_Gamepad extends RoboEaglesBase {

    private double MOTOR_SPEED_MULT = 0.7;
    private double ARM_SPEED_MULT = 0.5;

    private final double CLAW_INCREMENT = 0.02;
    // border values for Top Left Claw
//    private final double TOP_LEFT_CLAW_MIN = 0.03;

    // border values for Bottom Right Claw
    private final double BOTTOM_RIGHT_CLAW_MAX = 0.5;
    private final double BOTTOM_RIGHT_CLAW_MIN = 0;
    private final double BOTTOM_RIGHT_CLAW_INIT = BOTTOM_RIGHT_CLAW_MIN;

    // border values for Bottom Left Claw
    private final double BOTTOM_LEFT_CLAW_MAX = 1.0;
    private final double BOTTOM_LEFT_CLAW_MIN = 0.5;
    private final double BOTTOM_LEFT_CLAW_INIT = BOTTOM_LEFT_CLAW_MAX;

    // border values for "Elbow"
    private final double ARM_POWER = 0.5;
    private double topLeftClawPosition = TOP_LEFT_CLAW_INIT;
    private double topRightClawPosition = TOP_RIGHT_CLAW_INIT;
    private double bottomLeftClawPosition = BOTTOM_LEFT_CLAW_INIT;
    private double bottomRightClawPosition = BOTTOM_RIGHT_CLAW_INIT;

    public void runOpMode() {
        mapDevices();
        waitForStart();
        while (opModeIsActive()) {
            checkDriving();
            checkTopClaw();
            checkBottomClaw();
            checkArm();
            checkElbow();
            checkDroneLaunch();
            telemetry.update();
            sleep(10);
        }
    }

    void checkDriving() {
        if (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0)
            checkDrivingNormal();
        else if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0)
            checkDrivingStrafing();
        else {
            flDrive.setPower(0);
            frDrive.setPower(0);
            blDrive.setPower(0);
            brDrive.setPower(0);
        }
    }

    /**
     *  Method to check gamepad if we need to drive robot in a normal mode:
     *  all wheels rotate in the same direction
     *  Controlled by GamePad A:
     *  1) Axis Y on right stick controls movement speed
     *  2) Axis X on right stick adjust each side speed to make the robot turn
     *  Set the motors speed and put current execution to idle for a bit
     */
    void checkDrivingNormal() {
        double drive = -gamepad1.right_stick_y; // Read the Y-axis value of the left joystick and negate it
        double turn = gamepad1.right_stick_x; // Read the X-axis value of the left joystick
        double leftPower = Range.clip(drive + turn, -1, 1); // Calculate left motor power considering both drive and turn
        double rightPower = Range.clip(drive - turn, -1, 1); // Calculate right motor power considering both drive and turn

        leftPower *= MOTOR_SPEED_MULT;
        rightPower *= MOTOR_SPEED_MULT;

        telemetry.addData("DrivingNormal", "Joystick Drive: %f, Turn: %f", drive, turn);
        telemetry.addData("DrivingNormal", "Left Power: %f, Right Power: %f", leftPower, rightPower);

        moveSidesSame(0, leftPower, rightPower);
    }

    /**
     *`
     *  Method to check gamepad if we need to drive robot sideways:
     *  one set of diagonal wheels rotate in the same direction
     *  while another set of diagonal wheels rotate in opposite direction
     *  Controlled by GamePad A:
     *  1) Axis X on left stick controls movement speed
     *  2) Axis Y on left stick adjust each side speed to make the robot turn
     *  Set the motors speed and put current execution to idle for a bit
     */
    void checkDrivingStrafing() {
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;
        if (drive == 0 && turn == 0) return;

        double flBrPower = Range.clip(drive + turn, -1, 1); // Calculate left motor power considering both drive and turn
        double frBlPower = Range.clip(drive - turn, -1, 1); // Calculate right motor power considering both drive and turn

        // apply multiplier to the speed
        flBrPower *= MOTOR_SPEED_MULT;
        frBlPower *= MOTOR_SPEED_MULT;

        telemetry.addData("DrivingStrafing", "Joystick Drive: %f, Turn: %f", drive, turn);
        telemetry.addData("DrivingStrafing", "FlBr Power: %f, FrBl Power: %f", flBrPower, frBlPower);

        moveSidesDiagonal(0, flBrPower, frBlPower);
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
            bottomLeftClawPosition -= CLAW_INCREMENT ;
            if (bottomLeftClawPosition <= BOTTOM_LEFT_CLAW_MIN ) {
                bottomLeftClawPosition = BOTTOM_LEFT_CLAW_MIN;
            }
            // Right finger will go to maximum value and then stay at that position
            bottomRightClawPosition += CLAW_INCREMENT ;
            if (bottomRightClawPosition >= BOTTOM_RIGHT_CLAW_MAX) {
                bottomRightClawPosition = BOTTOM_RIGHT_CLAW_MAX;
            }
        }
        telemetry.addData("BottomClaw", "Left Claw: %f, Right Claw: %f", bottomLeftClawPosition, bottomRightClawPosition);
        // Move both servos to new position.  Assume servos are mirror image of each other.
        blClaw.setPosition(bottomLeftClawPosition);
        brClaw.setPosition(bottomRightClawPosition);
    }


    void checkArm() {
        double power = gamepad2.left_stick_y;
//        armMotor.setTargetPosition(armMotor.getTargetPosition() + (int) (power  * 10));
//        telemetry.addData("Arm", "Target Position: %d", armMotor.getTargetPosition());
        power *= ARM_SPEED_MULT;
        armMotor.setTargetPosition(armMotor.getTargetPosition() + (int) (power * 5));
        armMotor.setPower(power);
    }

    void checkElbow() {
        double power = gamepad2.right_stick_y; // Read the Y-axis value of the left joystick and negate it

        power *= ELBOW_SPEED_MULT;
        telemetry.addData("Elbow", "Power: %f", power);

        elbowMotor.setPower(power); // Set power to the front left motor
    }

    void checkDroneLaunch() {
        if (gamepad2.right_bumper) {
            telemetry.addData("DroneLaunch", "Drone launched called...");
            droneLauncher.setPosition(DRONE_LAUNCH_POSITION);
        }
    }
}