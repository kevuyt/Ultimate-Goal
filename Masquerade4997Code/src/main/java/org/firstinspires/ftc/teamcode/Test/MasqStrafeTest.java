package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqUtilities.StopCondition;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "MasqStrafeTest", group = "Autonomus")
public class MasqStrafeTest extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);

        while (!opModeIsActive()) {
            dash.create("Stalled: ", robot.intake.motor1.isStalled());
            dash.create("Velocity: ", robot.intake.motor1.getVelocity());
            dash.update();
        }
        waitForStart();
        if (robot.doubleBlock.stop()) {
            robot.go(new StopCondition() {
                @Override
                public boolean stop() {
                    return robot.doubleBlock.stop();
                }
            }, 45, Direction.LEFT, 0, Direction.FORWARD);
        }
        /*robot.strafe(48, Strafe.LEFT, POWER_HIGH);
        robot.strafe(48, Strafe.RIGHT, POWER_HIGH);*/
    }
}