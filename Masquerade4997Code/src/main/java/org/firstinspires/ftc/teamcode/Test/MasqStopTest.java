package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Autonomus.Constants;
import Library4997.MasqExternal.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 11/24/17.
 */
@Autonomous(name = "MasqStopTest", group = "Autonomus")
@Disabled
public class MasqStopTest extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("Distance: ", Double.toString(robot.ultra.getDistance()));
            dash.create("Distance: ", robot.ultra.getDistance() * 225);
            dash.update();
        }
        waitForStart();
        robot.stop(robot.ultra, .1, Direction.BACKWARD);
    }
}