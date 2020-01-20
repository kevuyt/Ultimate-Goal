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
@TeleOp(name = "XYMechTest", group = "MarkOne")
public class XYMechTest extends MasqLinearOpMode {
    public MarkOne robot = new MarkOne();
    private double hookPos;
    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        while (!opModeIsActive()) {
            dash.create("Hello, this is a test for a mech based gotoXY.");
            dash.create(robot.tracker);
            robot.tracker.updateSystem();
            dash.update();
        }

        waitForStart();
        //MasqWayPoint p = new MasqWayPoint(new MasqPoint(0, 20), 1);
        MasqWayPoint p1 = new MasqWayPoint(new MasqPoint(30, 30, -45));
        robot.xyPath(p1);
    }
}
