package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqWrappers.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * This is a basic template copy and paste this class for any auto,
 * refactor the file name to match the auto class title
 */

@Autonomous(name = "DefenseAuto", group = "Auto")
public class RelicDefAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.vuforiaInit();
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.create(robot.getTrackable());
            dash.update();
        }
        waitForStart();
        while (robot.getTrackable() ==  null){}
        if (robot.getTrackable() ==  LEFT)
            robot.drive(50);
        else if (robot.getTrackable() == RIGHT)
            robot.turn(90, Direction.LEFT);
    }
}