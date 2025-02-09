package org.firstinspires.ftc.teamcode;

import android.graphics.RenderNode;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import kotlin.collections.UArraySortingKt;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@TeleOp(name="RoboEagleOpMode", group="OpMode")
public class RoboEagleOpMode extends RoboEaglesBase {

//LET ME KNOW IF YOU NEED COMMENTS FOR BETTER LEARNING
public Servo left_hang, right_hang;

    void MapDevicesTesting() {
        Color = hardwareMap.get(ColorSensor.class,"colorSensor");
        distSensor = hardwareMap.get(DistanceSensor.class,"distSensor");
        brDrive = hardwareMap.dcMotor.get("br_motor");//OFFICIAL
        blDrive = hardwareMap.dcMotor.get("bl_motor");//OFFICIAL
        flDrive = hardwareMap.dcMotor.get("fl_motor");//OFFICIAL
        frDrive = hardwareMap.dcMotor.get("fr_motor");//OFFICIAL
        brClaw = hardwareMap.servo.get("br_claw");//OFFICIAL
        blClaw = hardwareMap.servo.get("bl_claw");//OFFICIAL
        middleElbow = hardwareMap.dcMotor.get("left_elbow1");
        left_hang = hardwareMap.servo.get("left_hang");
        right_hang = hardwareMap.servo.get("right_hang");
        bottomrClaw = hardwareMap.servo.get("specl_claw");//OFFICIAL
        bottomlClaw = hardwareMap.servo.get("specr_claw");//OFFICIAL
        leftElbow = hardwareMap.dcMotor.get("left_elbow");//OFFICIAL
        rightElbow = hardwareMap.dcMotor.get("right_elbow");//OFFICIAL
        rightArm = hardwareMap.get(DcMotorEx.class, "right_arm");//OFFICIAL
        //hangWheel =  hardwareMap.get(DcMotorEx.class, "wheels");
        rightElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//OFFICIAL
        leftElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//OFFICIAL
        //extImu = hardwareMap.get(IMU.class,"extgyro");
        //Rev9AxisImuOrientationOnRobot extImu = new Rev9AxisImuOrientationOnRobot(hardwareMap.get(I2cDevice.class,"extgyro"));
        extImu = hardwareMap.get(IMU.class,"extgyro");
        gyro = new RevIMU(hardwareMap);
        gyro.init();
        resetIMU();
    }

    private double MOTOR_SPEED_MULT = 0.3;//used to be 0.7
    public DcMotor leftElbow, rightElbow, middleElbow;
    public DcMotorEx leftArm, rightArm;
    public Servo bottomrClaw;
    public Servo bottomlClaw;
    RevIMU gyro;
    //public DcMotorEx hangWheel;
    private final double CLAW_INCREMENT = 0.02;
    private final double BOTTOM_RIGHT_CLAW_MAX = 0.8;
    private final double BOTTOM_RIGHT_CLAW_MIN = 0;
    private final double BOTTOM_RIGHT_CLAW_INIT = BOTTOM_RIGHT_CLAW_MAX;
    private final double BOTTOM_LEFT_CLAW_MAX = 0.8;
    private final double BOTTOM_LEFT_CLAW_MIN = 0;
    private final double BOTTOM_LEFT_CLAW_INIT = BOTTOM_LEFT_CLAW_MIN;
    private double bottomLeftClawPosition = BOTTOM_LEFT_CLAW_INIT;
    private double bottomRightClawPosition = BOTTOM_RIGHT_CLAW_INIT;
    private double ARM_SPEED_MULT = 0.5;
    public final double ELBOW_SPEED_MULT = 0.75;
    public double power_arm, current_arm_pos;
    boolean Hang = false;
    boolean start_hang_wheel = false;
    private ColorSensor Color;
    private DistanceSensor distSensor;
    private IMU extImu;
    private IMU.Parameters parameters;
    private Orientation angles;
    //private double MOTOR_SPEED_MULT = 0.7;
    boolean hang_servo_value = false;
    public void runOpMode() {
        MapDevicesTesting();

        waitForStart();

       while (opModeIsActive()) {
           if (Hang == false) {
               checkArm();
           }
            newSensorTele();
            //HangServo();
            checkDriving();
            ArcadeDrive();
            checkBaseClaw();
            checkElbow();
            Hanging();
            checkBottomClaw();
            HangServo();
            RobotOrientation();

            telemetry.update();
            sleep(10);
        }



    }
    double ELBOW_SPEED_MULT_NEW;
     void checkElbow() {
        double power = gamepad2.right_stick_y; // Read the Y-axis value of the left joystick and negate it
         double prev_power = 0;

        // if (power_arm >0 ) {
         /*if(current_arm_pos < 0)
         {
             ELBOW_SPEED_MULT_NEW = 60;
         }
         else
         {
             ELBOW_SPEED_MULT_NEW = 5;
         }*/
         //ELBOW_SPEED_MULT_NEW = 0.5;
         ELBOW_SPEED_MULT_NEW = 0.8;
        power *= ELBOW_SPEED_MULT_NEW;
        telemetry.addData("Elbow left", "Power: %f", leftElbow.getPower());
         telemetry.addData("Elbow right ", "Power: %f", rightElbow.getPower());
         telemetry.addData("Elbow center", "Power: %f", middleElbow.getPower());

        //rightElbow.setPower(power);
       if(hang_servo_value == true)
         {
             //ELBOW_SPEED_MULT_NEW = 1;
             power = power*100;

         }
             telemetry.addData("Elbow", "Power: %f", power);
             leftElbow.setPower(-power);
             rightElbow.setPower(power);
             middleElbow.setPower(-power);



    }
    boolean armIncrease = false;
    void HangServo() {


        if (gamepad2.right_trigger>0) {//open
            hang_servo_value = true;
            armIncrease = true;
            left_hang.setPosition(0.1);//used to 0.3
            right_hang.setPosition(0.9);//used to 0.6
            sleep(500);
        }

        if (gamepad2.left_trigger>0) {//close
            hang_servo_value = true;
            armIncrease = true;
            left_hang.setPosition(0.9);//used to 0.6
            right_hang.setPosition(0.1);//used to 0.3
            sleep(500);
        }
    }//
    void checkDriving_turn() {
        if ((gamepad1.left_stick_y != 0) &&
                (gamepad1.right_stick_x ==0))
            checkDrivingNormal();
        else if (gamepad1.right_stick_x != 0)
            //checkDrivingStrafing();
            checkDrivingNormal_turn();
        else {
            flDrive.setPower(0);
            frDrive.setPower(0);
            blDrive.setPower(0);
            brDrive.setPower(0);
        }
    }


    void checkDriving() {

        if (gamepad1.left_stick_x != 0 || gamepad1.right_stick_y != 0)
            checkDrivingNormal();
        else if (gamepad1.right_stick_x != 0 || gamepad1.left_stick_y != 0)
            //checkDrivingStrafing();
            checkDrivingNormal();
        else {
            flDrive.setPower(0);
            frDrive.setPower(0);
            blDrive.setPower(0);
            brDrive.setPower(0);
        }
    }
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
    public void ArcadeDrive() {
        double right_stick_x = gamepad1.right_stick_x;
        //double right_stick_x = 1
        if  (Math.abs(right_stick_x) > 0.1) {
            /*if(right_stick_x < 0) {right_stick_x = -1;}
            else if(right_stick_x > 0) {right_stick_x = 1;}
            else {right_stick_x = 0;}*/

            flDrive.setPower(-right_stick_x);
            frDrive.setPower(-right_stick_x);
            blDrive.setPower(right_stick_x);
            brDrive.setPower(right_stick_x);
            telemetry.addData("Strafing Each Motor Power", "FL: %f, Fr: %f, Bl: %f, Br: %f ", flDrive.getPower(), frDrive.getPower(), blDrive.getPower(), brDrive.getPower());
            //telemetry.addData("StrafingEncoderPower", "FL: %f, Fr: %f, Bl: %f, Br: %f ", flDrive.getCurrentPosition(), frDrive.getCurrentPosition(), blDrive.getCurrentPosition(), brDrive.getCurrentPosition());
            //original code
            /*flDrive.setPower((left_stick_y-left_stick_x)-right_stick_x);
            frDrive.setPower(-1 * ((left_stick_y+left_stick_x)+right_stick_x));
            blDrive.setPower((left_stick_y-left_stick_x)+right_stick_x);
            brDrive.setPower(-1 * ((left_stick_y-left_stick_x)-right_stick_x));
             */
        } 
    }

    void DrivingTest() {
        double drive = gamepad1.left_stick_y;
        double turn = -gamepad1.left_stick_x;
        double leftPower = Range.clip(drive + turn, -1, 1);
        double rightPower = Range.clip(drive - turn, -1, 1);
        flDrive.setPower(leftPower);
        blDrive.setPower(leftPower);
        frDrive.setPower(rightPower*(-1));
        brDrive.setPower(rightPower*(-1));
    }

    void checkDrivingNormal() {
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;
        //double turn = 0;
        double leftPower = Range.clip(drive - turn, -1, 1);//used to be positive
        double rightPower = Range.clip(drive + turn, -1, 1);//used to be negative

        //double leftPower = Range.clip(drive + turn, -1, 1);
        //double rightPower = -leftPower;

        leftPower *= MOTOR_SPEED_MULT*2.6;
        rightPower *= MOTOR_SPEED_MULT*2.6;


        telemetry.addData("2026 DrivingNormal", "Joystick Drive: %f, Turn: %f", drive, turn);
        telemetry.addData("DrivingNormal", "Left Power: %f, Right Power: %f", leftPower, rightPower);

        moveSidesSame(0, leftPower, -1 * (rightPower));
        //moveSidesSame(0, -1*(leftPower), rightPower);


    }
    void checkDrivingNormal_turn() {
        double drive = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;

        //double turn = 0;

        double leftPower = Range.clip(drive - turn, -1, 1);//used to be positive
        double rightPower = Range.clip(drive + turn, -1, 1);//used to be negative

        //double leftPower = Range.clip(drive + turn, -1, 1);
        //double rightPower = -leftPower;
        leftPower *= MOTOR_SPEED_MULT*2;
        rightPower *= MOTOR_SPEED_MULT*2;

        telemetry.addData("2026 DrivingNormal", "Joystick Drive: %f, Turn: %f", drive, turn);
        telemetry.addData("DrivingNormal", "Left Power: %f, Right Power: %f", leftPower, rightPower);
        if(turn < 0)
            moveSidesSame_turn(0,0, leftPower, -1 * (rightPower));
        else
            moveSidesSame_turn(1, 0, leftPower, -1 * (rightPower));

        //moveSidesSame(0, -1*(leftPower), rightPower);


    }

    //boolean Hang;
    void Hanging () {
        if (!Hang) {
           // Hang = gamepad1.b;
            Hang = gamepad1.left_bumper && gamepad1.right_bumper;
        }

        telemetry.addData("Hanging", "Hang: %b, ArmPower: %f", Hang, power_arm);
        if (Hang) {
            rightArm.setTargetPosition(rightArm.getTargetPosition() + (int) (power_arm));
            rightArm.setPower(power_arm/1.5);
            //rightArm.setPower(power_arm);

        }
    }

    void checkBaseClaw() {
        boolean open_servo = gamepad1.x;    // Close fingers
        boolean close_servo = gamepad1.y;     // open fingers
        telemetry.addData("BottomClaw", "Open: %b, Close: %b", open_servo, close_servo);
        if (open_servo) {
            brClaw.setPosition(0);//NEVER CHANGE THIS CODE!!!!!!!
            blClaw.setPosition(0.45);//0.45
            //sleep(500);
            /*brClaw.setPosition(0.55);//NEVER CHANGE THIS CODE!!!!!!!
            blClaw.setPosition(0.25);
            sleep(500);*/
        }

        if (close_servo) {
            brClaw.setPosition(0.30);//NEVER CHANGE THIS CODE!!!!!!!
            blClaw.setPosition(0.15);
            //sleep(500);
            /*brClaw.setPosition(0.15);//NEVER CHANGE THIS CODE!!!!!!!
            blClaw.setPosition(0.75);
            sleep(500);*/
        }
    }
    public void diagStraf() {
        double right_stick_x = gamepad1.right_stick_x;
        double right_stick_y = gamepad1.right_stick_y;
        //double right_stick_x = 1
        if (Math.abs(right_stick_y)>0.1 && Math.abs(right_stick_x)>0.1) {
            if(right_stick_y >0.1) {
                flDrive.setPower(-right_stick_x);
                frDrive.setPower(0);
                blDrive.setPower(0);
                brDrive.setPower(right_stick_x);
            }
            else
            {
                flDrive.setPower(0);
                frDrive.setPower(-right_stick_x);
                blDrive.setPower(right_stick_x);
                brDrive.setPower(0);
            }
        }
        else if (Math.abs(right_stick_x) > 0.1) {
            /*if(right_stick_x < 0) {right_stick_x = -1;}
            else if(right_stick_x > 0) {right_stick_x = 1;}
            else {right_stick_x = 0;}*/

            flDrive.setPower(-right_stick_x);
            frDrive.setPower(-right_stick_x);
            blDrive.setPower(right_stick_x);
            brDrive.setPower(right_stick_x);
            telemetry.addData("Strafing Each Motor Power", "FL: %f, Fr: %f, Bl: %f, Br: %f ", flDrive.getPower(), frDrive.getPower(), blDrive.getPower(), brDrive.getPower());
            //telemetry.addData("StrafingEncoderPower", "FL: %f, Fr
        }
    }
    public void RobotOrientation(){
        //imu = hardwareMap.get(BNO055IMU.class,"imu");
        double targetAngle = gyro.getAbsoluteHeading();
        telemetry.addData("Imu, imu = %f", targetAngle);
        gyro.getHeading();
        telemetry.addData("heading = %f", gyro.getHeading());
    }
    void checkBottomClaw() {
        boolean open_servo = gamepad2.x;    // Close fingers
        boolean close_servo = gamepad2.y;     // open fingers
        telemetry.addData("BottomClaw", "Open: %b, Close: %b", open_servo, close_servo);
        if (open_servo) {
            bottomrClaw.setPosition(0);//NEVER CHANGE THIS CODE!!!!!!!
            bottomlClaw.setPosition(0.45);
            //sleep(500);
            /*brClaw.setPosition(0.55);//NEVER CHANGE THIS CODE!!!!!!!
            blClaw.setPosition(0.25);
            sleep(500);*/
        }

        if (close_servo) {
            bottomrClaw.setPosition(0.45);//NEVER CHANGE THIS CODE!!!!!!!
            bottomlClaw.setPosition(0);
            //sleep(500);
            /*brClaw.setPosition(0.15);//NEVER CHANGE THIS CODE!!!!!!!
            blClaw.setPosition(0.75);
            sleep(500);*/
        }
    }

    int hang_mult = 1;
    void checkArm() {
        if (armIncrease == true) {
          hang_mult = 20;
        }
        power_arm = -gamepad2.left_stick_y * hang_mult;

        /*if (power_arm != 0) {
            //rightArm.setPower(100);
            rightArm.setPower();
        }*/
        rightArm.setTargetPosition(rightArm.getTargetPosition() + (int) (power_arm*10));
        telemetry.addData("arm, arm = %f",rightArm.getPower());

        if (power_arm > -0) {
            rightArm.setPower(power_arm*0.8);// extend
        }
        else if (power_arm < 0){
            rightArm.setPower(power_arm*1); // down
        }
        else {
            rightArm.setPower(0.1);
        }
        //rightArm.setPower(power_arm);

    }

}  // end class




