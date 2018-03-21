package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 11/9/17.
 */
@Autonomous(name = "MasqTurnTest", group = "Autonomus")
public class MasqTurnTest extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_OUT);
        robot.sleep(1000);
        if (robot.jewelColorBlue.isBlue()) robot.blueRotator.setPosition(ROTATOR_BLUE_SEEN);
        else robot.blueRotator.setPosition(ROTATOR_BLUE_NOT_SEEN);
        robot.sleep(250);
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
        robot.sleep(100);
        while (!opModeIsActive()) {
            dash.create(robot.jewelColorBlue);
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        robot.drive(48, POWER_OPTIMAL);
        robot.turnRelative(90, Direction.LEFT);
        robot.turnRelative(90, Direction.RIGHT);
        robot.drive(48, POWER_OPTIMAL, Direction.BACKWARD);
    }
}