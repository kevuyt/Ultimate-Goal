package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;
import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPointLegacy;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.LEFT;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.MIDDLE;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "Blue Two Stone", group = "MarkOne")
public class BlueTwoStone extends MasqLinearOpMode{
    private MarkOne robot = new MarkOne();
    private SkystonePosition position;
    private List<MasqPoint> stones = new ArrayList<>();
    private MasqWayPointLegacy
            bridge1 = new MasqWayPointLegacy(new MasqPoint(-25,19,90),5,0.9),
            bridge2 = new MasqWayPointLegacy(new MasqPoint(-59,20,90),3,0.7),
            foundation = new MasqWayPointLegacy(new MasqPoint(-90,28,90),3,0.4),
            park = new MasqWayPointLegacy(new MasqPoint(-34, 19, -90),0.5,0);

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
        robot.sideGrabber.rightMid(1);
        robot.xyPathLegacy(bridge1, bridge2, foundation);
        robot.sideGrabber.rightLowMid(0);
        robot.sideGrabber.rightOpen(0);
        robot.xyPathLegacy(bridge2,bridge1,
                new MasqWayPointLegacy(new MasqPoint(stone2.getX(),stone2.getY()-5,stone2.getH()),0.5,0),
                new MasqWayPointLegacy(stone2,0.5,0.15));
        robot.driveTrain.stopDriving();
        robot.sideGrabber.rightDown(1);
        robot.sideGrabber.rightClose(1);
        robot.sideGrabber.rightMid(1);
        robot.xyPathLegacy(bridge1,bridge2,foundation);
        robot.sideGrabber.rightLowMid(0);
        robot.sideGrabber.rightOpen(0.5);
        robot.turnRelative(90, Direction.LEFT,1.5);
        robot.gotoXY(robot.tracker.getGlobalX(), robot.tracker.getGlobalY() + 5,
                robot.tracker.getHeading(), 1, 0.5, 1.5);
        robot.foundationHook.lower();
        sleep(1.25);
        robot.gotoXY(new MasqPoint(robot.tracker.getGlobalX(), robot.tracker.getGlobalY()-25,
                robot.tracker.getHeading()),2,0.5,2.5);
        robot.gotoXY(new MasqPoint(-72, 0, -90),3,0.5,1.5);
        robot.sideGrabber.rightClose(0);
        robot.foundationHook.raise();
        sleep(1.25);
        robot.gotoXY(new MasqPoint(robot.tracker.getGlobalX()-15,robot.tracker.getGlobalY(),robot.tracker.getHeading()));
        robot.xyPathLegacy(new MasqWayPointLegacy(new MasqPoint(robot.tracker.getGlobalX(),19,-90),0.5,0.2),park);
    }
}