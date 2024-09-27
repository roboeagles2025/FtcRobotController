package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@TeleOp(name="DeepDrivingONLY", group="OpMode")
public class DeepDrivingONLY extends RoboEaglesBase {


    private double MOTOR_SPEED_MULT = 0.7;

    void MapDevicesTesting() {
        brDrive = hardwareMap.dcMotor.get("br_motor");
        blDrive = hardwareMap.dcMotor.get("bl_motor");
        flDrive = hardwareMap.dcMotor.get("fl_motor");
        frDrive = hardwareMap.dcMotor.get("fr_motor");

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
    public final double ELBOW_SPEED_MULT = 0.75;
    //private double MOTOR_SPEED_MULT = 0.7;

    public void runOpMode() {
        MapDevicesTesting();

        waitForStart();
        while (opModeIsActive()) {
            checkDriving();
            //checkBottomClaw();
            telemetry.update();
            sleep(100);
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

        moveSidesSame(0, leftPower, -1 * (rightPower));

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
}





