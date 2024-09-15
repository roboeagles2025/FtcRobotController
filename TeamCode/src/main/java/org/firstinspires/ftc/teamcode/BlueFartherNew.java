package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import java.util.List;

@Autonomous(name = "Blue Farther New", group = "Autonomous")
public class BlueFartherNew extends RoboEaglesAutonomousBaseNew {

    @Override
    public void runOpMode() {

        autonomousStart();
    }

    void spikeMarkLeft() {
        // Move to the TeamProp
        driveStraightPID(13);
        turnPID(50);
        driveStraightPID(6);

        // Move back from TeamProp to the middle of tile
        driveStraightPID(-10);
        turnPID(-55);

        // Move forward to get in the middle of the field and turn toward the board
        driveStraightPID(32);
        turnPID(90);

        // Move to the board, adjust a bit during the movement its direction
        driveStraightPID(60);
        turnPID(15);
        driveStraightPID(30);

        // Open the bottom claw to release the pixel
        blClaw.setPosition(BOTTOM_LEFT_CLAW_OPEN);
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_OPEN);
        sleep(1000);
    }


    void spikeMarkMiddle() {
        // Move to the TeamProp
        driveStraightPID(22);
        turnPID(-20);
        driveStraightPID(7.5);

        // Move back from TeamProp to the middle of tile
        driveStraightPID(-5);

        // Turn toward the board and move all the way forward to the board through second tile
        turnPID(107);
        driveStraightPID(60);
        turnPID(-90);
        driveStraightPID(20);
        turnPID(90);
        driveStraightPID(20);


        // Open the bottom claw to release the pixel
        blClaw.setPosition(BOTTOM_LEFT_CLAW_OPEN);
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_OPEN);
        sleep(1000);
    }

    void spikeMarkRight() {
        // Move to the TeamProp
        driveStraightPID(16);
        turnPID(-55);
        driveStraightPID(3);

        // Move back from TeamProp to the middle of tile
        driveStraightPID(-6);
        turnPID(55);

        // Move forward to get in the middle of the field and turn toward the board
        driveStraightPID(37);
        turnPID(90);

        // Move to the board, adjust a bit during the movement its direction
        driveStraightPID(60);
        turnPID(20);
        driveStraightPID(35);

        // Open the bottom claw to release the pixel
        blClaw.setPosition(BOTTOM_LEFT_CLAW_OPEN);
        brClaw.setPosition(BOTTOM_RIGHT_CLAW_OPEN);
        sleep(1000);
    }



}


