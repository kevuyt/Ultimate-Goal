package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;
import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint.PointMode.MECH;
import static Library4997.MasqResources.MasqHelpers.Direction.BACKWARD;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.RIGHT;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "RedIndependent", group = "MarkOne")
public class RedIndependent extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    private SkystonePosition position;
    private int stoneCount = 1;
    private List<MasqWayPoint> stones = new ArrayList<>();
    private MasqWayPoint
            bridge1 = new MasqWayPoint().setPoint(24, 20, 90).setSwitchMode(MECH),
            bridge2 = new MasqWayPoint().setPoint(59, 25, 90).setSwitchMode(MECH).setOnComplete(() -> {
                robot.sideGrabber.leftClose(0);
                robot.sideGrabber.leftUp(0);
            }),
            park = new MasqWayPoint().setPoint(35,24, -90).setMaxVelocity(1).setMinVelocity(0),
            foundationOne = new MasqWayPoint().setPoint(86, 32, 90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.leftSlightClose(0);
                robot.sideGrabber.leftLowMid(0);
            }),
            foundationTwo = new MasqWayPoint().setPoint(88, 32, 90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.leftSlightClose(0);
                robot.sideGrabber.leftLowMid(0);
            }),
            foundationThree = new MasqWayPoint().setPoint(92, 32, 90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.leftSlightClose(0);
                robot.sideGrabber.leftLowMid(0);
            });

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        robot.initCamera(hardwareMap);

        stones.add(null);
        // MEASURED VALUES, DO NOT EDIT
        stones.add(new MasqWayPoint().setPoint(12, 29, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(4, 29, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(-4, 29, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));

        stones.add(new MasqWayPoint().setPoint(-13, 29,90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(-21, 29, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(-29, 29, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2).setDriveCorrectionSpeed(0.04));

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

        //idk why but the first stone always needs a lil more in the x, it goes to the right position tho
        runStones[0] = runStones[0].setX(runStones[0].getX() + 3);

        runSimultaneously(
                () -> mainAuto(runStones),
                () -> robot.cv.stop()
        );
    }

    private void mainAuto(MasqWayPoint... stones) {
        grabStone(stones[0].setSwitchMode(MECH).setOnComplete(() -> {
            robot.sideGrabber.leftClose(1);
            robot.sideGrabber.leftUp(0.5);
        }), foundationOne);
        robot.tracker.setDrift(0, 3);
        grabStone(stones[1], foundationTwo);
        robot.tracker.setDrift(0, 6);
        grabStone(stones[2], foundationThree);
        foundationPark();
    }

    private void grabStone(MasqWayPoint stone, MasqWayPoint foundation) {
        if (stoneCount == 1) robot.xyPath(4, stone);
        else robot.xyPath(9, bridge2, bridge1.setOnComplete(() -> {
            robot.sideGrabber.leftOpen(0);
            robot.sideGrabber.leftDown(0);
        }), stone.setOnComplete(() -> {
            double closeSleep = 1, rotateSleep = 0;
            robot.sideGrabber.leftClose(closeSleep);
            robot.sideGrabber.leftUp(rotateSleep);
        }));
        robot.driveTrain.setVelocity(0);
        if (stoneCount == 4) {
            bridge2.setH(180).setOnComplete(() -> {
                robot.sideGrabber.rightDown(0);
                robot.sideGrabber.rightOpen(0);
            });
        }
        robot.xyPath(5, exitStone(), bridge1.setOnComplete(null), bridge2, foundation);
        robot.driveTrain.setVelocity(0);
        stoneCount++;
    }

    private void foundationPark() {
        robot.turnAbsolute(170,1);
        robot.drive(10,0.5, BACKWARD,1);
        robot.foundationHook.lower();
        sleep();
        MasqWayPoint p1 = new MasqWayPoint().setPoint(-92,5, 160)
                .setOnComplete(() -> {
                    robot.turnAbsolute(90);
                    robot.foundationHook.raise();
                    robot.stop(1);
                }).setMinVelocity(0);
        robot.xyPath(6, p1, park);
        robot.stop(0.5);
        /*robot.turnAbsolute(180,1);
        robot.drive(7,1.25,BACKWARD,1);
        robot.foundationHook.lower();
        sleep();
        MasqWayPoint p1 = new MasqWayPoint().setPoint(60,0, -90)
                .setDriveCorrectionSpeed(0.2).setMinVelocity(0).setOnComplete(() -> robot.foundationHook.raise());
        MasqWayPoint p2 = new MasqWayPoint().setPoint(90,20, -60)
                .setDriveCorrectionSpeed(1);
        robot.xyPath(5, p1, p2, park);
        robot.stop(0.5);*/
    }
    private MasqWayPoint exitStone() {
        return new MasqWayPoint()
                .setPoint(robot.tracker.getGlobalX(), robot.tracker.getGlobalY() - 6, -robot.tracker.getHeading());
    }

    private MasqWayPoint[] middleStones() {
        return new MasqWayPoint[]{stones.get(2), stones.get(5), stones.get(3), stones.get(1)};
    }
    private MasqWayPoint[] leftStones() {
        return new MasqWayPoint[]{stones.get(3), stones.get(6), stones.get(2), stones.get(1)};
    }
    private MasqWayPoint[] rightStones() {
        return new MasqWayPoint[]{stones.get(1), stones.get(4), stones.get(3), stones.get(2)};
    }
}