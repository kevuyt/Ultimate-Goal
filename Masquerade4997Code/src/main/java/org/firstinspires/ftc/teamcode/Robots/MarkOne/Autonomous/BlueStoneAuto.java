package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;
import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.LEFT;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.MIDDLE;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "Blue Stone", group = "MarkOne")
public class BlueStoneAuto extends MasqLinearOpMode{
    private MarkOne robot = new MarkOne();
    private SkystonePosition position;
    private List<MasqPoint> stones = new ArrayList<>();
    private MasqWayPoint
            bridge1 = new MasqWayPoint(new MasqPoint(-25,19,90),5,0.9),
            bridge2 = new MasqWayPoint(new MasqPoint(-59,20,90),3,0.7),
            foundation = new MasqWayPoint(new MasqPoint(-90,31,90),4,0.4);
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        stones.add(null);

        stones.add(new MasqPoint(-17,30,90));
        stones.add(new MasqPoint(-7,29,90));
        stones.add(new MasqPoint(0,29,90));

        stones.add(new MasqPoint(7.5,29,90));
        stones.add(new MasqPoint(12,30,90));
        stones.add(new MasqPoint(22,30,90));
        robot.cv.start();

        while(!opModeIsActive()) {
            position = CVInterpreter.getPosition(robot.cv.detector);
            dash.create("Skystone Position: ", position);
            dash.update();
        }

        waitForStart();
        robot.sideGrabber.rightUp(0);
        robot.sideGrabber.leftUp(0);
        robot.sideGrabber.rightOpen(0);
        robot.sideGrabber.leftClose(0);
        if (position == LEFT) runSimultaneously(
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
        );
    }
    private void mainAuto(MasqPoint stone1, MasqPoint stone2) {
        robot.gotoXY(stone1);
        robot.sideGrabber.rightDown(1);
        robot.sideGrabber.rightClose(1);
        robot.sideGrabber.rightMid(0);
        robot.xyPath(bridge1, bridge2, foundation);
        robot.sideGrabber.rightLowMid(0);
        robot.sideGrabber.rightOpen(0);
        robot.xyPath(bridge2,bridge1,
                new MasqWayPoint(stone2,0.5,0));
        robot.sideGrabber.rightDown(1);
        robot.sideGrabber.rightClose(1);
        robot.sideGrabber.rightMid(0);
        robot.xyPath(bridge1,bridge2,foundation);
        robot.sideGrabber.rightLowMid(0);
        robot.sideGrabber.rightOpen(0);
        sleep();
        robot.turnRelative(90, Direction.LEFT);
        robot.gotoXY(robot.tracker.getGlobalX(), robot.tracker.getGlobalY() + 7,
                robot.tracker.getHeading(), 1.5, 0.5, 1);
        robot.gotoXY(new MasqPoint(robot.tracker.getGlobalX()-6,robot.tracker.getGlobalY(),
                robot.tracker.getHeading()),1.5,0.5,1);
        robot.foundationHook.lower();
        sleep(1);
        robot.gotoXY(new MasqPoint(robot.tracker.getGlobalX(), robot.tracker.getGlobalY()-25,
                robot.tracker.getHeading()));
        robot.gotoXY(new MasqPoint(-60, 0, -90),2,0.5,0.5);
        robot.foundationHook.raise();
        robot.gotoXY(new MasqPoint(-84, 24, -90));
        robot.gotoXY(new MasqPoint(-34, 21, -90));
    }
}