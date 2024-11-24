package org.firstinspires.ftc.teamcode.threadTeleop;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoboEaglesAutoBase2425;
import org.firstinspires.ftc.teamcode.threadTeleop.ThreadHandler;

public class ThreadBase extends RoboEaglesAutoBase2425 {
    MotorGroup lGroup, rGroup;
    public com.arcrobotics.ftclib.controller.PIDController pidDriveLeft, pidDriveRight, pidRotate;
    com.arcrobotics.ftclib.controller.PIDController pidArm;
    RevIMU gyro;
    public final double DRIVE_SPEED_MULTIPLIER = 0.75;

    MotorEx brDriveEx, blDriveEx, flDriveEx, frDriveEx;
    private final double LEFT_SIDE_MULTIPLIER = 1;
    private final double RIGHT_SIDE_MULTIPLIER = 1;
    public DcMotor leftElbow, rightElbow;
    public DcMotorEx rightArm;
    private long TimeToRun;
    private final int ONE_SECOND = 1000;
    private final double DEGREE_PER_SEC = 380;
    private final long go_straight_time_const = 25000;
    private final double DEGREE_PER_SEC_NEW = 44;
    public double power_arm, current_arm_pos;
    double elbow_power = 5;
    int turn_value = 1;

    public void MapDevicesThread() {
        flDriveEx = new MotorEx(hardwareMap, "fl_motor");
        frDriveEx = new MotorEx(hardwareMap, "fr_motor");
        blDriveEx = new MotorEx(hardwareMap, "bl_motor");
        brDriveEx = new MotorEx(hardwareMap, "br_motor");
        brClaw = hardwareMap.servo.get("br_claw");
        blClaw = hardwareMap.servo.get("bl_claw");
        leftPower = 10;
        rightPower = 10;
        leftElbow = hardwareMap.dcMotor.get("left_elbow");//OFFICIAL
        rightElbow = hardwareMap.dcMotor.get("right_elbow");//OFFICIAL
        rightArm = hardwareMap.get(DcMotorEx.class, "right_arm");//OFFICIAL
        //rightElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//OFFICIAL
        //leftElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//OFFICIAL
        MapDevicesPIDs();
    }
    Thread openClaw = new Thread(new Thread() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds()<300) {
        }
            brClaw.setPosition(0.7);
            blClaw.setPosition(0.2);
        }
});
}






