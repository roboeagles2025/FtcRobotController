package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Autonomous Blue Farther", group = "Autonomous")
@Disabled
public class AutonomousBlueFarther extends RoboEaglesAutonomousBase {
    @Override
    public void runOpMode() {
        backboardInches = 82;
        teamColor = "BLUE";
        autonomousStart();
    }

    void placePurplePixelLeft() {
        liftArm(0.3);
        // initialize bottom claw to have it half opened
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_CENTER);
        blClaw.setPosition(BOTTOM_LEFT_CLAW_CENTER);
        // Move 23.5 inches forward
        driveStraightSmoothly(0.5, 10);
        sleep(1000);
        // Rotate 70 degrees to the right so we won't go directly to the team prop
        turnPID(35, 0.5);
        sleep(1000);
        // Move few inches forward
        driveStraightSmoothly(0.5, 13);
        sleep(1000);
        turnPID(50, 0.5);
    }

    void placePurplePixelMiddle() {
        liftArm(0.3);
        // initialize bottom claw to have it half opened
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_CENTER);
        blClaw.setPosition(BOTTOM_LEFT_CLAW_CENTER);
        sleep(1000);
        // Move 23.5 inches forward
        driveStraightSmoothly(0.5, 23);
        sleep(1000);
        // turn 25 degree to the left to avoid team prop
        turnPID(-25, 1.5);
        // Move few inches forward
        driveStraightSmoothly(0.5, 10);
        sleep(1000);
        // Open left side of the claw completely while keeping right side half open so we don't shove team prop
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_CLOSED);
        blClaw.setPosition(BOTTOM_LEFT_CLAW_CENTER + 0.075);
        sleep(1000);
        // Move few inches backward
        driveStraightSmoothly(0.5, -7);
        sleep(1000);
        // close the claw
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_CLOSED);
        blClaw.setPosition(BOTTOM_LEFT_CLAW_CLOSED);
        sleep(1000);
        // Rotate 115 degrees to the right so the robot faces backdrop
    }

    void placePurplePixelRight() {
        liftArm(0.3);
        // initialize bottom claw to have it half opened
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_CENTER);
        blClaw.setPosition(BOTTOM_LEFT_CLAW_CENTER);
        // Move 23.5 inches forward
        driveStraightSmoothly(0.5, 25);
        sleep(1000);
        // Rotate 75 degrees to the left so we won't go directly to the team prop
        turnPID(-95, 0.5);
        sleep(1000);
        // Move few inches forward
        driveStraightSmoothly(0.3, 7);
        sleep(1000);
        // Open left side of the claw completely while keeping right side half open so we don't shove team prop
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_CLOSED);
        blClaw.setPosition(BOTTOM_LEFT_CLAW_CENTER + 0.075);
        sleep(1000);
        // Move few inches backward
        driveStraightSmoothly(0.5, -15);
        sleep(1000);
        // close the claw
//        brClaw.setPosition(BOTTOM_RIGHT_CLAW_OPEN);
//        blClaw.setPosition(BOTTOM_LEFT_CLAW_OPEN);
//        sleep(1000);
//        liftArm(0);
        // Rotate 130 degrees to the right so the robot faces backdrop
//        turnPID(-155, 0.7);
//        sleep(1000);
        // Open left side of the claw completely while keeping right side half open so we don't shove team prop
//        brClaw.setPosition(BOTTOM_LEFT_CLAW_CLOSED);
//        blClaw.setPosition(BOTTOM_LEFT_CLAW_CENTER - 0.075);
//        sleep(1000);
        // Move few inches backward
//        driveStraightSmoothly(0.5, -3);
//        sleep(1000);
    }
}