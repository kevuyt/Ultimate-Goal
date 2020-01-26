package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "Blue Full", group = "MarkOne")
public class BlueFullAuto extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    private SkystonePosition position;
    private List<MasqPoint> stones = new ArrayList<>();
    private MasqWayPoint
            bridge1 = new MasqWayPoint(new MasqPoint(-25, 19, 90), 5, 0.9),
            bridge2 = new MasqWayPoint(new MasqPoint(-59, 20, 90), 3, 0.7),
            foundation = new MasqWayPoint(new MasqPoint(-90, 28, 90), 3, 0.4),
            park = new MasqWayPoint(new MasqPoint(-34, 19, -90), 0.5, 0);

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();

        stones.add(null);

        stones.add(new MasqPoint(-17, 30, 90));
        stones.add(new MasqPoint(-7, 29, 90));
        stones.add(new MasqPoint(0, 30, 90));

        stones.add(new MasqPoint(7.5, 28, 90));
        stones.add(new MasqPoint(15, 30, 90));
        stones.add(new MasqPoint(22, 28.5, 90));

        //robot.cv.start();

        while (!opModeIsActive()) {
            //position = CVInterpreter.getPosition(robot.cv.detector);
            dash.create("Skystone Position: ", position);
            dash.update();
        }

        waitForStart();

        robot.sideGrabber.rightUp(0);
        robot.sideGrabber.leftUp(0);
        robot.sideGrabber.rightClose(0);
        robot.sideGrabber.leftClose(0);

        //mainAuto(stones.get(1), stones.get(4));
        robot.gotoXY(stones.get(1));
        MasqWayPoint bridge1kk = new MasqWayPoint(new MasqPoint(-47, 20, -90), 5, 0.7);
        MasqWayPoint bridge2kk = new MasqWayPoint(new MasqPoint(-57, 20, -90), 5, 0).setMaxVelocity(0.7f);
        MasqWayPoint bridge3kk = new MasqWayPoint(new MasqPoint(-47, 25, -90), 5, 0);
        robot.xyPath(20, bridge1kk, bridge2kk);

        /*if (position == LEFT) runSimultaneously(
                () -> mainAuto(stones.get(1), stones.get(4)),
                () -> robot.cv.stop()
        );
        else if (position == MIDDLE) runSimultaneously(
                () -> mainAuto(stones.get(2), stones.get(5)),
                () -> robot.cv.stop()
        );
        else runSimultaneously(
                () -> mainAuto(stones.get(3), stones.get(6)),
                () -> robot.cv.stop()
        );*/
    }

    private void mainAuto(MasqPoint stone1, MasqPoint stone2) {
        robot.gotoXY(stone1);
        //robot.sideGrabber.rightDown(1);
        //robot.sideGrabber.rightClose(1);
        //robot.sideGrabber.rightMid(1);
        robot.xyPath(bridge1, bridge2, foundation);
        //robot.sideGrabber.rightLowMid(0);
        //robot.sideGrabber.rightOpen(0);
        robot.xyPath(bridge2, bridge1,
                new MasqWayPoint(new MasqPoint(stone2.getX(), stone2.getY() - 5, stone2.getH()), 0.5, 0),
                new MasqWayPoint(stone2, 0.5, 0.15));
        robot.driveTrain.stopDriving();
        //robot.sideGrabber.rightDown(1);
        //robot.sideGrabber.rightClose(1);
        //robot.sideGrabber.rightMid(1);
        robot.xyPath(bridge1, bridge2, foundation);
        //robot.sideGrabber.rightLowMid(0);
        //robot.sideGrabber.rightOpen(0.5);

        robot.turnRelative(90, Direction.LEFT, 1.5);
        robot.gotoXY(robot.tracker.getGlobalX(), robot.tracker.getGlobalY() + 5,
                robot.tracker.getHeading(), 1, 0.5, 1);
        robot.foundationHook.lower();
        sleep(1);
        foundationPark();
    }

    private void foundationPark() {
        MasqWayPoint p1 = new MasqWayPoint(new MasqPoint(-68, 5, 90), 1,
                0, 0.7, 10, 0.01);
        MasqWayPoint p2 = new MasqWayPoint(new MasqPoint(-68, 5, 90), 1,
                0, 0.7, 10, 0.01);
        robot.xyPath(2, p1, p2);
        robot.foundationHook.raise();
        robot.xyPath(park);
        dash.create("Completed Path");
        dash.update();
        sleep(10);
    }
}