package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;
import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint.PointMode.MECH;
import static Library4997.MasqResources.MasqUtils.velocityAutoController;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.RIGHT;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.MIDDLE;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "RedFourNeut", group = "MarkOne")
public class RedFourNeut extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    private SkystonePosition position;
    private int stoneCount = 0;
    private List<MasqWayPoint> stones = new ArrayList<>();
    private MasqWayPoint
            bridge1 = new MasqWayPoint().setPoint(24, 20, 90).setSwitchMode(MECH),
            bridge2 = new MasqWayPoint().setPoint(59, 25, 90).setSwitchMode(MECH).setOnComplete(() -> {
                robot.sideGrabber.leftClose(0);
                robot.sideGrabber.leftUp(0);
            }),
            foundationOne = new MasqWayPoint().setPoint(86, 32, 90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.leftSlightClose(0);
                robot.sideGrabber.leftLowMid(0);
            }),
            foundationTwo = new MasqWayPoint().setPoint(88, 32, 90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.leftSlightClose(0);
                robot.sideGrabber.leftLowMid(0);
            }),
            foundationThree = new MasqWayPoint().setPoint(92, 32, 90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.leftLowMid(0);
                robot.sideGrabber.leftOpen(1);
            }),
            foundationFour = new MasqWayPoint().setPoint(48, 32, 180).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.leftLowMid(0);
                robot.sideGrabber.leftOpen(1);
            });

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        robot.initCamera(hardwareMap);

        stones.add(null);

        stones.add(new MasqWayPoint().setPoint(15, 29.5, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(8, 29.5, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(0, 29.5, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));

        stones.add(new MasqWayPoint().setPoint(-9, 29.5,90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(-15, 29.5, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(-25, 29.5, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2).setDriveCorrectionSpeed(0.04));

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

        if (position == RIGHT) runSimultaneously(
                () -> mainAuto(stones.get(1), stones.get(4),stones.get(3)),
                () -> robot.cv.stop()
        );
        else if (position == MIDDLE) runSimultaneously(
                () -> mainAuto(stones.get(2), stones.get(5),stones.get(3)),
                () -> robot.cv.stop()
        );
        else runSimultaneously(
                () -> mainAuto(stones.get(3), stones.get(6),stones.get(1)),
                () -> robot.cv.stop()
        );
    }

    private void mainAuto(MasqWayPoint... stones) {
        grabStone(stones[0].setSwitchMode(MECH).setOnComplete(() -> {
            robot.sideGrabber.leftClose(1);
            robot.sideGrabber.leftUp(0.5);
        }), foundationOne);
        robot.tracker.setDrift(0, -3);
        grabStone(stones[1], foundationTwo);
        robot.tracker.setDrift(0, -6);
        grabStone(stones[2], foundationThree);
        robot.tracker.setDrift(0, -9);
        grabStone(stones[3], foundationFour);
        foundationPark();
    }

    private void grabStone(MasqWayPoint stone, MasqWayPoint foundation) {
        if (stoneCount == 0) robot.xyPath(4, stone);
        else robot.xyPath(9, bridge2, bridge1.setOnComplete(() -> {
            robot.sideGrabber.leftOpen(0);
            robot.sideGrabber.leftDown(0);
        }), stone.setOnComplete(() -> {
            double closeSleep = 1, rotateSleep = 0.5;
            robot.sideGrabber.leftClose(closeSleep);
            robot.sideGrabber.leftUp(rotateSleep);
        }));
        robot.driveTrain.setVelocity(0);
        robot.xyPath(5, bridge1.setOnComplete(null), bridge2, foundation);
        robot.driveTrain.setVelocity(0);
        stoneCount++;
    }

    private void foundationPark() {
        robot.sideGrabber.leftClose(0);
        robot.sideGrabber.leftUp(0);
        velocityAutoController.setKp(0.001);
        MasqWayPoint p3 = new MasqWayPoint().setPoint(40,24, 90);
        robot.xyPath(29 - timeoutClock.seconds(), p3);
        //obot.drive(60, Direction.BACKWARD);
    }
}