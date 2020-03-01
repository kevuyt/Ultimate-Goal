package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;
import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint.PointMode.MECH;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.RIGHT;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.MIDDLE;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "RedCollab", group = "MarkOne")
public class RedCollab extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    private SkystonePosition position;
    private int stoneCount = 1, maxStones = 4;
    private List<MasqWayPoint> stones = new ArrayList<>();
    private MasqWayPoint
            bridge1Entry = new MasqWayPoint().setPoint(24, 20, 90).setSwitchMode(MECH).setDriveCorrectionSpeed(0.1),
            bridge1Exit = new MasqWayPoint().setPoint(24, 22, 90).setSwitchMode(MECH).setDriveCorrectionSpeed(0.1),
            bridge2 = new MasqWayPoint().setPoint(59, 25, 90).setSwitchMode(MECH).setOnComplete(() -> {
                robot.sideGrabber.leftClose(0);
                robot.sideGrabber.leftUp(0);
            }),
            park = new MasqWayPoint().setPoint(35,24, 180).setMinVelocity(0),
            foundationOne = new MasqWayPoint().setPoint(86, 32, 90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.leftSlightClose(0);
                robot.sideGrabber.leftLowMid(0);
            }),
            foundationTwo = new MasqWayPoint().setPoint(88, 32, 90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.leftSlightClose(0);
                robot.sideGrabber.leftLowMid(0);
            }),
            foundationThree = new MasqWayPoint().setPoint(90, 32, 90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.leftSlightClose(0);
                robot.sideGrabber.leftLowMid(0);
            }),
            foundationFour = new MasqWayPoint().setPoint(60, 20, 180).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                runSimultaneously(
                        () -> robot.stop(1),
                        () -> {
                            robot.sideGrabber.leftLowMid(0);
                            robot.sideGrabber.leftOpen(1);
                        }
                );
            });

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        robot.initCamera(hardwareMap);

        stones.add(null);
        // MEASURED VALUES, DO NOT EDIT
        stones.add(new MasqWayPoint().setPoint(12, 28, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(4, 28, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(-4, 28, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));

        stones.add(new MasqWayPoint().setPoint(-13, 28,90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(-21, 28, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(-29, 28, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2).setDriveCorrectionSpeed(0.04));

        while (!opModeIsActive()) {
            position = CVInterpreter.getRed(robot.cv.detector);
            dash.create("Skystone Position: " + position);
            dash.update();
        }

        waitForStart();

        timeoutClock.reset();
        robot.sideGrabber.leftDown(0);
        robot.sideGrabber.rightUp(0);
        robot.sideGrabber.leftOpen(0);
        robot.sideGrabber.rightClose(0);
        robot.foundationHook.raise();
        MasqWayPoint[] runStones;

        if (position == RIGHT) runStones = rightStones();
        else if (position == MIDDLE) runStones = middleStones();
        else runStones = leftStones();

        runSimultaneously (
                () -> mainAuto(runStones),
                () -> robot.cv.stop()
        );
    }

    private void mainAuto(MasqWayPoint... stones) {
        int index = 0;
        while (index < maxStones) {
            stones[index] = stones[index].setOnComplete(() -> {
                double closeSleep = 1, upSleep = 0;
                runSimultaneously(
                        () -> robot.stop(closeSleep + upSleep),
                        () -> {
                            robot.sideGrabber.leftClose(closeSleep);
                            robot.sideGrabber.leftUp(upSleep);
                        }
                );
            }).setSwitchMode(MECH).setMinVelocity(0);
            index++;
        }
        // negative drift move it to the right.
        robot.tracker.setDrift(-3, 0);
        grabStone(stones[0], foundationOne);
        robot.tracker.setDrift(-3, 1.5);
        grabStone(stones[1], foundationTwo);
        robot.tracker.setDrift(-3, 3);
        bridge1Exit = bridge1Exit.setX(bridge1Exit.getX() + 10);
        bridge1Entry = bridge1Entry.setY(bridge1Entry.getY() - 2);
        grabStone(stones[2], foundationThree);
        robot.tracker.setDrift(-5, 4.5);
        grabStone(stones[3], foundationFour);
        foundationPark();
    }

    private void grabStone(MasqWayPoint stone, MasqWayPoint foundation) {
        if (stoneCount == 1) robot.xyPath(4, stone);
        else robot.xyPath(9, bridge2, bridge1Exit.setOnComplete(() -> {
            robot.sideGrabber.leftOpen(0);
            robot.sideGrabber.leftDown(0);
        }), stone);
        if (stoneCount == 4) {
            bridge2.setH(180).setOnComplete(() -> {
                robot.sideGrabber.rightDown(0);
                robot.sideGrabber.rightOpen(0);
            });
        }
        robot.xyPath(5, exitStone(), bridge1Entry, bridge2, foundation);
        robot.driveTrain.setVelocity(0);
        stoneCount++;
    }

    private void foundationPark() {
        robot.xyPath(5, park);
        robot.stop(5 - timeoutClock.seconds());
    }

    private MasqWayPoint exitStone() {
        return new MasqWayPoint()
                .setPoint(robot.tracker.getGlobalX(), robot.tracker.getGlobalY() - 6, -robot.tracker.getHeading());
    }

    private MasqWayPoint[] middleStones() {
        return new MasqWayPoint[] {
                stones.get(2),
                stones.get(5),
                stones.get(3),
                stones.get(1)
        };
    }
    private MasqWayPoint[] leftStones() {
        return new MasqWayPoint[] {
                stones.get(3),
                stones.get(6),
                stones.get(2),
                stones.get(1)
        };
    }
    private MasqWayPoint[] rightStones() {
        return new MasqWayPoint[] {
                stones.get(1),
                stones.get(4),
                stones.get(3),
                stones.get(2)
        };
    }
}