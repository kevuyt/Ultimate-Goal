package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import MasqLibrary.MasqOdometry.MasqWayPoint;
import MasqLibrary.MasqResources.MasqLinearOpMode;

import static MasqLibrary.MasqRobot.OpMode.TELEOP;

/**
 * Created by Keval Kataria on 12/30/2020
 */

@TeleOp(group = "Test")
public class XYPathTester extends MasqLinearOpMode {
    private final Osiris robot = new Osiris();

    @Override
    public void runLinearOpMode() {
        robot.init(TELEOP);

        while(!opModeIsActive()) {
            dash.create(robot.tracker);
            dash.update();

            robot.tracker.updateSystem();

            if(isStopRequested()) break;
        }

        waitForStart();
        timeoutClock.reset();

        robot.xyPath(new MasqWayPoint(24, 24, 0).setTimeout(30 - timeoutClock.seconds()).setMinVelocity(0));
        sleep((long) (30e3 - timeoutClock.milliseconds()));

    }
}