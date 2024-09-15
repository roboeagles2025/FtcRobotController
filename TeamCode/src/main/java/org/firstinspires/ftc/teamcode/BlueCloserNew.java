package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Closer New", group = "Autonomous")
public class BlueCloserNew extends RoboEaglesAutonomousBaseNew {

    @Override
    public void runOpMode() {

        autonomousStart();
    }

    void spikeMarkLeft() {
        // Move to the TeamProp
        driveStraightPID(10);
        turnPID(35);
        driveStraightPID(11);

        // Move back from TeamProp to the middle of tile
        driveStraightPID(-11);
        turnPID(-35);

        // Move back to the wall and turn toward the board and move to the board to park
        driveStraightPID(-6);
        turnPID(90);
        driveStraightPID(40);

        // Open the bottom claw to release the pixel
        blClaw.setPosition(BOTTOM_LEFT_CLAW_OPEN);
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_OPEN);
        sleep(1000);
    }

    void spikeMarkMiddle() {
        // Move to the TeamProp
        driveStraightPID(22);
        turnPID(-20);
        driveStraightPID(5);

        // Move back from TeamProp to the middle of tile
        driveStraightPID(-5);
        turnPID(20);

        // Move back to the wall and turn toward the board and move to the board to park
        driveStraightPID(-18);
        turnPID(90);
        driveStraightPID(40);

        // Open the bottom claw to release the pixel
        blClaw.setPosition(BOTTOM_LEFT_CLAW_OPEN);
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_OPEN);
        sleep(1000);
    }
    void spikeMarkRight() {
        // Move to the TeamProp
        driveStraightPID(16);
        turnPID(-55);
        driveStraightPID(5);

        // Move back from TeamProp to the middle of tile
        driveStraightPID(-5);
        turnPID(55);

        // Move back to the wall and turn toward the board and move to the board to park
        driveStraightPID(-12);
        turnPID(90);
        driveStraightPID(40);

        // Open the bottom claw to release the pixel
        blClaw.setPosition(BOTTOM_LEFT_CLAW_OPEN);
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_OPEN);
        sleep(1000);
    }

}


