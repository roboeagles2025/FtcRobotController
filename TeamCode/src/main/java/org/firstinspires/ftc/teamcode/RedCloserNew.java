package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Closer New", group = "Autonomous")
public class RedCloserNew extends RoboEaglesAutonomousBaseNew {

    @Override
    public void runOpMode() {

        autonomousStart();
    }

    void spikeMarkLeft() {
        // Move to the TeamProp
        driveStraightPID(16);
        turnPID(55);
        driveStraightPID(6);

        // Move back from TeamProp to the middle of tile
        driveStraightPID(-6);
        turnPID(-55);

        // Move back to the wall and turn toward the board and move to the board to park
        driveStraightPID(-14);
        turnPID(-90);
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
        driveStraightPID(6);

        // Move back from TeamProp to the middle of tile
        driveStraightPID(-6);
        turnPID(20);

        // Move back to the wall and turn toward the board and move to the board to park
        driveStraightPID(-18);
        turnPID(-90);
        driveStraightPID(40);

        // Open the bottom claw to release the pixel
        blClaw.setPosition(BOTTOM_LEFT_CLAW_OPEN);
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_OPEN);
        sleep(1000);
    }

    void spikeMarkRight() {
        // Move to the TeamProp
        driveStraightPID(14);
        turnPID(-35);
        driveStraightPID(6);

        // Move back from TeamProp to the middle of tile
        driveStraightPID(-6);
        turnPID(35);

        // Move back to the wall and turn toward the board and move to the board to park
        driveStraightPID(-10);
        turnPID(-90);
        driveStraightPID(40);

        // Open the bottom claw to release the pixel
        blClaw.setPosition(BOTTOM_LEFT_CLAW_OPEN);
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_OPEN);
        sleep(1000);
    }

}


