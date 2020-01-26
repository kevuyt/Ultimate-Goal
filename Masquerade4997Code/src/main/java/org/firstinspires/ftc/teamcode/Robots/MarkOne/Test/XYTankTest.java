package org.firstinspires.ftc.teamcode.Robots.MarkOne.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 2020-01-12.
 * Project: MasqLib
 */
@TeleOp(name = "XYTankTest", group = "MarkOne")
public class XYTankTest extends MasqLinearOpMode {
    public MarkOne robot = new MarkOne();
    private double hookPos;
    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        robot.sideGrabber.teleReset();
        robot.foundationHook.lower();
        while (!opModeIsActive()) {
            dash.create("Hello, this is a test for a tank based gotoXY.");
            dash.create(robot.tracker);
            dash.create(robot.foundationHook.leftHook.getPosition());
            dash.create(robot.foundationHook.rightHook.getPosition());
            robot.tracker.updateSystem();
            dash.update();
        }
        waitForStart();

        MasqWayPoint p0 = new MasqWayPoint(new MasqPoint(0, 10, 0), 1,
                0, 0.7, 10, 0.01);
        MasqWayPoint p1 = new MasqWayPoint(new MasqPoint(20, 30, 90), 1,
                0, 0.7, 10, 0.02);
        MasqWayPoint p2 = new MasqWayPoint(new MasqPoint(35, 60, 0), 1,
                0, 0.5, 10, 0.02);
        MasqWayPoint p3 = new MasqWayPoint(new MasqPoint(40, 80, 0), 1,
                0, 0.3,10, 0.02);

        robot.xyPathTank(20, p0, p1, p2, p3);
    }
}
