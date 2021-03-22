package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import MasqueradeLibrary.MasqOdometry.MasqWayPoint;
import MasqueradeLibrary.MasqResources.MasqLinearOpMode;

import static MasqueradeLibrary.MasqRobot.OpMode.TELEOP;

/**
 * Created by Keval Kataria on 12/30/2020
 */

@TeleOp(name = "XYPathTester", group = "Test")
public class XYPathTester extends MasqLinearOpMode {
    private Osiris robot = new Osiris();

    @Override
    public void runLinearOpMode() {
        robot.init(TELEOP);

        while(!opModeIsActive()) {
            dash.create(robot.tracker);
            dash.update();

            if(isStopRequested()) break;
        }

        waitForStart();
        timeoutClock.reset();

        robot.xyPath(new MasqWayPoint(24, 24, 0).setTimeout(30 - timeoutClock.seconds()).setMinVelocity(0));
        sleep((long) (30000 - timeoutClock.milliseconds()));

    }
}