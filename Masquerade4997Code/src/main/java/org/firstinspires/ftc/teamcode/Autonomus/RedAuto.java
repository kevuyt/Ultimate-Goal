package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;
import SubSystems4997.SubSystems.Flipper.Position;
import SubSystems4997.SubSystems.Flipper.Grip;

;

/**
 * Created by Archish on 3/25/18.
 */
@Autonomous(name = "RedAuto", group = "Autonomus")
public class RedAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        robot.flipper.setPosition(Grip.CLAMP);
        robot.drive(28, POWER_OPTIMAL, Direction.BACKWARD);
        robot.flipper.flip(0);
        robot.turnAbsolute(45, Direction.RIGHT);
        robot.flipper.setPosition(Grip.OUT);
        robot.flipper.setPosition(Position.IN);
        robot.go(40, 45, Direction.BACKWARD, 0, Direction.RIGHT);
    }
}