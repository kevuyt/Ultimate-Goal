package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 11/3/17.
 */
@Autonomous(name = "MasqTurnTest", group = "Autonomus")
public class MasqTurnTest extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        robot.turn(110, Direction.LEFT);
        robot.turn(-110, Direction.LEFT);
        robot.turn(110, Direction.LEFT);
        robot.turn(-110, Direction.LEFT);
    }
}