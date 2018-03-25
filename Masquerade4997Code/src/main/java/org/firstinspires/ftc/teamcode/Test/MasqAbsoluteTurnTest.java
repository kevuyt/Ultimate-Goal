package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 3/8/18.
 */
@Autonomous(name = "MasqAbsolute", group = "Autonomus")
public class MasqAbsoluteTurnTest extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.imu.adjustAngle(robot.imu.getAbsoluteHeading()));
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        robot.turnAbsolute(60, Direction.RIGHT);
        dash.create("Turn 1 Complete Turn two");
        dash.update();
        sleep(5);
        robot.turnAbsolute(60, Direction.RIGHT);
        dash.create("Turn 2 Complete Turn three");
        dash.update();
        robot.turnAbsolute(0, Direction.RIGHT);
        robot.turnAbsolute(60, Direction.LEFT);
        robot.turnAbsolute(135, Direction.LEFT );
    }
}