package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;
import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.LEFT;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.MIDDLE;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "Blue Stone", group = "MarkOne")
@Disabled
public class BlueStoneAuto extends MasqLinearOpMode{
    private MarkOne robot = new MarkOne();
    private SkystonePosition position;
    private List<MasqPoint> stones = new ArrayList<>();
    private MasqWayPoint
            bridge1 = new MasqWayPoint(new MasqPoint(-25,19,90),5,0.9),
            bridge2 = new MasqWayPoint(new MasqPoint(-59,20,90),3,0.7),
            foundation = new MasqWayPoint(new MasqPoint(-94,31,90),4,0.4);

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();

        stones.add(null);

        stones.add(new MasqPoint(-17,30,90));
        stones.add(new MasqPoint(-7,29,90));
        stones.add(new MasqPoint(0,30,90));

        stones.add(new MasqPoint(7.5,28,90));
        stones.add(new MasqPoint(15,30,90));
        stones.add(new MasqPoint(22,28.5,90));

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
                () -> mainAuto(stones.get(1), stones.get(4), stones.get(2), stones.get(3)),
                () -> robot.cv.stop()
        );
        else if (position == MIDDLE) runSimultaneously(
                () -> mainAuto(stones.get(2), stones.get(5), stones.get(1), stones.get(3)),
                () -> robot.cv.stop()
        );
        else runSimultaneously(
                    () -> mainAuto(stones.get(3), stones.get(6), stones.get(1), stones.get(2)),
                    () -> robot.cv.stop()
            );
    }
    private void mainAuto(MasqPoint stone1, MasqPoint stone2, MasqPoint stone3, MasqPoint stone4) {
        robot.gotoXY(stone1);
        robot.sideGrabber.rightDown(1);
        robot.sideGrabber.rightClose(1);
        robot.sideGrabber.rightMid(1);
        robot.xyPath(bridge1, bridge2,foundation);
        robot.sideGrabber.rightLowMid(0);
        robot.sideGrabber.rightOpen(0);
        robot.xyPath(bridge2,bridge1,
                new MasqWayPoint(stone2,0.5,0));
        robot.sideGrabber.rightDown(1);
        robot.sideGrabber.rightClose(1);
        robot.sideGrabber.rightMid(1);
        robot.xyPath(bridge1,bridge2,foundation);
        robot.sideGrabber.rightLowMid(0);
        robot.sideGrabber.rightOpen(0);
        robot.xyPath(bridge2,bridge1,new MasqWayPoint(stone3,0.5,0));
        robot.sideGrabber.rightDown(1);
        robot.sideGrabber.rightClose(1);
        robot.sideGrabber.rightMid(1);
        robot.xyPath(bridge1,bridge2,foundation);
        robot.sideGrabber.rightLowMid(0);
        robot.sideGrabber.rightOpen(0);
        robot.gotoXY(new MasqPoint(-34, 19, -90));
    }
}