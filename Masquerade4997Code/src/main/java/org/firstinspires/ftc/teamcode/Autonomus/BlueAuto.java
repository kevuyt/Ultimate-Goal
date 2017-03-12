package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import BasicLib4997.MasqLinearOpMode;
import BasicLib4997.MasqMotors.MasqRobot.MasqRobot;

/**
 * Blue Autonomous
 */

@Autonomous(name = "BlueAuto", group = "Test")
public class BlueAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot);
            dash.update();
        }
        waitForStart();
        dash.createSticky(robot);
        robot.setAllianceColor(MasqRobot.AllianceColor.BLUE);
    }
}