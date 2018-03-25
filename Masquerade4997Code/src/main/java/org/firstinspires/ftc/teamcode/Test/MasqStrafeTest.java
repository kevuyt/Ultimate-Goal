package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "MasqStrafeTest", group = "Autonomus")
public class MasqStrafeTest extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.go(48, 45, Direction.LEFT, 45, Direction.LEFT);
        //robot.go(12, 45, Direction.LEFT);
        /*robot.strafe(48, Strafe.LEFT, POWER_HIGH);
        robot.strafe(48, Strafe.RIGHT, POWER_HIGH);*/
    }
}