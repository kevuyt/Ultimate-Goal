package org.firstinspires.ftc.teamcode.Robots.MarkOne.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 2020-01-12.
 * Project: MasqLib
 */
@TeleOp(name = "XYTankTest", group = "MarkOne")
@Disabled
public class XYTankTest extends MasqLinearOpMode {
    public MarkOne robot = new MarkOne();
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

        MasqWayPoint p0 = new MasqWayPoint().setPoint(-0,18,0).setModeSwitchRadius(5)
                .setPointSwitchRadius(5);
        MasqWayPoint p1 = new MasqWayPoint().setPoint(-36,18,-90).setModeSwitchRadius(5)
                .setPointSwitchRadius(5).setMinVelocity(0);

        robot.xyPath(20, p0, p1);
    }
}
