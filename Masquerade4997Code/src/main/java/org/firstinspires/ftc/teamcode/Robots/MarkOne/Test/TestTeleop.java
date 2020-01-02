package org.firstinspires.ftc.teamcode.Robots.MarkOne.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqPositionTracker;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/17/2019
 */
@TeleOp(name = "TestTeleop", group = "MarkOne")
public class TestTeleop extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeTeleop();

        while (!opModeIsActive()) {
            dash.create("X: ",robot.tracker.getGlobalX());
            dash.create("Y: ",robot.tracker.getGlobalY());
            robot.tracker.updateSystem(MasqPositionTracker.DeadWheelPosition.BOTH_PERPENDICULAR);
            dash.update();
        }

        while(opModeIsActive()) {
            robot.MECH(controller1,1, 0.3);
            dash.create(robot.tracker);
            dash.update();
            robot.sideGrabber.reset();
            robot.tracker.updateSystem(MasqPositionTracker.DeadWheelPosition.BOTH_PERPENDICULAR);
        }
    }
}
