package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 1/7/18.
 */
@Autonomous(name = "TestAuto", group = "Autonomus")
public class TestAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        robot.drive(200, POWER_OPTIMAL, Direction.BACKWARD);
        robot.turn(90, Direction.LEFT);
        robot.flipper.setPosition(FLIPPER_OUT);
    }
}