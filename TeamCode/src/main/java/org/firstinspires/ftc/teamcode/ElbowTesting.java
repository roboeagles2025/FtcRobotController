package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@TeleOp(name="ElbowTEST", group="OpMode")
public class ElbowTesting extends RoboEaglesBase {




    void MapDevicesTesting() {
        //brDrive = hardwareMap.dcMotor.get("br_motor");
        //blDrive = hardwareMap.dcMotor.get("bl_motor");
        //flDrive = hardwareMap.dcMotor.get("fl_motor");
        //frDrive = hardwareMap.dcMotor.get("fr_motor");
        //brClaw = hardwareMap.servo.get("br_claw");
        //blClaw = hardwareMap.servo.get("bl_claw");
       // leftElbow = hardwareMap.get(DcMotorEx.class, "left_elbow");
       // rightElbow = hardwareMap.get(DcMotorEx.class, "right_elbow");
        leftElbow = hardwareMap.dcMotor.get("left_elbow");
        rightElbow = hardwareMap.dcMotor.get("right_elbow");
        //leftArm = hardwareMap.dcMotor.get("left_arm");
        //rightArm = hardwareMap.dcMotor.get("right_arm");
        //leftArm = hardwareMap.get(DcMotorEx.class, "left_arm");
        rightArm = hardwareMap.get(DcMotorEx.class, "right_arm");

        rightElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private double MOTOR_SPEED_MULT = 0.7;
    public DcMotor leftElbow, rightElbow;
    public DcMotorEx leftArm, rightArm;
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
          //checkElbow();
          //checkArm();

       while (opModeIsActive()) {
            //checkArm();
            //checkDriving();
            //checkBottomClaw();
            //checkElbow();
            checkElbowNEW();
            telemetry.update();
            sleep(10);
        }



    }

     void checkElbowNEW() {
        double power = gamepad2.right_stick_y; // Read the Y-axis value of the left joystick and negate it
         double prev_power = 0;
        double ELBOW_SPEED_MULT_NEW = 10;
        power *= ELBOW_SPEED_MULT_NEW;
        telemetry.addData("Elbow", "Power: %f", power);

        //rightElbow.setPower(power);
         if(power <= 0)
         {
             leftElbow.setPower(power);
             rightElbow.setPower(-power);
         }
         /*
         else if(power == 0)
         {
             power = 1;
             leftElbow.setPower(power);
             rightElbow.setPower(-power);
         }else if(prev_power < 0)
         {

             power = 1;
             leftElbow.setPower(power);
             rightElbow.setPower(-power);

         }

         prev_power = power;

          */
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
    }

    void checkArm() {
        double power = -gamepad2.left_stick_x;
        //rightArm.setTargetPosition(armMotor.getTargetPosition() + (int) (power  * 10));
        //telemetry.addData("RightArm", "Target Position: %d", armMotor.getTargetPosition());
        //power *= ARM_SPEED_MULT;
        //double power = 0.5;
        rightArm.setTargetPosition(rightArm.getTargetPosition() + (int) (power * 5));
        //armMotor.setTargetPosition(armMotor.getTargetPosition() + (int) (power * 5));
        //leftArm.setPower(power);
        rightArm.setPower(power);
        //sleep(2000);
        //leftArm.setPower(0);
        //rightArm.setPower(-power);
        //sleep(2000);
    }
    void checkElbowOLD() {
       // double power = gamepad2.right_stick_y; // Read the Y-axis value of the left joystick and negate it

        //power *= ELBOW_SPEED_MULT; //THIS IS ORIGINAL VERSION
        double power = 3;
        telemetry.addData("Elbow", "Power: %f", power);

        //leftElbow.setPower(-power);
        rightElbow.setPower(power);
        sleep(400);
        //while (opModeIsActive()) {
        for(int cnt = 0; cnt <2; cnt++){
            rightElbow.setPower(-power);
            sleep(100);
            rightElbow.setPower(power);
            //rightElbow.setPower(-power);
            sleep(100);
        }
        rightElbow.setPower(0);

    }

    // Step through the list of detections and display info for each one.


}  // end class




