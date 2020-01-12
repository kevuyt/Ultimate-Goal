package org.firstinspires.ftc.teamcode.Robots.MarkOne.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 2020-01-02.
 * Project: MasqLib
 */
@Autonomous(name = "gotoXYTank Test" , group = "MarkOne")
public class TestAutoFour extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    private MasqWayPoint inital = new MasqWayPoint(new MasqPoint(0, 0, 0), 1, 0.5);
    private MasqWayPoint one = new MasqWayPoint(new MasqPoint(20, 20, 0), 4, 0.5);
    private MasqWayPoint two = new MasqWayPoint(new MasqPoint(20, 0, 0), 1, 0);
    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);
        robot.initializeAutonomous();

        while (!opModeIsActive()) {
            dash.create("x: ",robot.tracker.getGlobalX());
            dash.create("Y: ",robot.tracker.getGlobalY());
            robot.tracker.updateSystem();
            dash.update();
        }

        waitForStart();
        robot.xyPath(inital, one, two);
        //robot.gotoXYTank(-30,30, 5, 120, 1);
    }
}
