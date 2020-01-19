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

import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter
        .SkystonePosition;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter
        .SkystonePosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter
        .SkystonePosition.RIGHT;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "Red Full", group = "MarkOne")
public class RedFullAuto extends MasqLinearOpMode{
    private MarkOne robot = new MarkOne();
    private SkystonePosition position;
    private List<MasqPoint> stones = new ArrayList<>();
    private MasqWayPoint
            bridge1 = new MasqWayPoint(new MasqPoint(25,19,-90),4,0.9),
            bridge2 = new MasqWayPoint(new MasqPoint(59,20,-90),4,0.7),
            foundation = new MasqWayPoint(new MasqPoint(85.5,32.5,-90),4,
                    0.4),
            park = new MasqWayPoint(new MasqPoint(30, 20, 90),0.5,0);
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        stones.add(null);

        stones.add(new MasqPoint(8.75,30,-90));
        stones.add(new MasqPoint(0.5,30,-90));
        stones.add(new MasqPoint(-6,30,-90));

        stones.add(new MasqPoint(-14,31,-90));
        stones.add(new MasqPoint(-22,30,-90));
        stones.add(new MasqPoint(-29,31,-90));
        robot.cv.start();

        while(!opModeIsActive()) {
            position = CVInterpreter.getPosition(robot.cv.detector);
            dash.create("Skystone Position: ", position);
            dash.update();
        }

        waitForStart();
        robot.sideGrabber.leftUp(0);
        robot.sideGrabber.rightUp(0);
        robot.sideGrabber.leftOpen(0);
        robot.sideGrabber.rightClose(0);
        if (position == RIGHT) runSimultaneously(
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
        robot.sideGrabber.leftDown(1);
        robot.sideGrabber.leftClose(1);
        robot.sideGrabber.leftMid(1);
        robot.xyPath(bridge1, bridge2, foundation);
        robot.sideGrabber.leftLowMid(0);
        robot.sideGrabber.leftOpen(0);
        robot.xyPath(bridge2,bridge1,
                new MasqWayPoint(new MasqPoint(stone2.getX(),stone2.getY()-5,stone2.getH()),
                        0.5,0),
                new MasqWayPoint(stone2,0.5,0));
        robot.sideGrabber.leftDown(1);
        robot.sideGrabber.leftClose(1);
        robot.sideGrabber.leftMid(1);
        robot.xyPath(bridge1,bridge2,foundation);
        robot.driveTrain.stopDriving();
        robot.sideGrabber.leftLowMid(0);
        robot.sideGrabber.leftOpen(0);
        sleep();
        robot.turnRelative(90, Direction.RIGHT);
        robot.gotoXY(robot.tracker.getGlobalX(), robot.tracker.getGlobalY() + 3,
                robot.tracker.getHeading(), 1.25, 0.5, 1.5);
        robot.foundationHook.lower();
        sleep(1);
        robot.gotoXY(new MasqPoint(robot.tracker.getGlobalX(), robot.tracker.getGlobalY() - 25,
                robot.tracker.getHeading()),1.5,0.5,1.5);
        robot.gotoXY(new MasqPoint(72, 0, 90),3,0.5,1.5);
        robot.sideGrabber.leftClose(0);
        robot.foundationHook.raise();
        robot.xyPath(new MasqWayPoint(new MasqPoint(robot.tracker.getGlobalX(),20,90),
                0.5,0.25),park);
    }
}