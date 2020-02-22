package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;
import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint.PointMode.MECH;
import static Library4997.MasqSensors.MasqClock.Resolution.SECONDS;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.LEFT;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.MIDDLE;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "Blue Max Stones", group = "MarkOne")
public class BlueMaxStones extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    private SkystonePosition position;
    private List<MasqWayPoint> stones = new ArrayList<>();
    private MasqWayPoint
            bridge1 = new MasqWayPoint().setPoint(-24, 20, -90).setSwitchMode(MECH),
            bridge2 = new MasqWayPoint().setPoint(-59, 24, -90).setSwitchMode(MECH),
            foundationOne = new MasqWayPoint().setPoint(-86, 32, -90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.rightSlightClose(0);
                robot.sideGrabber.rightLowMid(0);
            }),
            foundationTwo = new MasqWayPoint().setPoint(-88, 38, -90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.rightSlightClose(0);
                robot.sideGrabber.rightLowMid(0);
            }),
            foundationThree = new MasqWayPoint().setPoint(-92, 35, -90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.rightSlightClose(0);
                robot.sideGrabber.rightLowMid(0);
            }),
            foundationFour = new MasqWayPoint().setPoint(-82, 27, -180).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.rightSlightClose(0);
                robot.sideGrabber.rightLowMid(0);
            }),
            park = new MasqWayPoint().setPoint(-45,28, 90).setDriveCorrectionSpeed(0.2).setLookAhead(5);

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        robot.initCamera(hardwareMap);

        stones.add(null);

        stones.add(new MasqWayPoint().setPoint(-17.5, 29.5, -90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2).setDriveCorrectionSpeed(0.04));
        stones.add(new MasqWayPoint().setPoint(-8.5, 29.5, -90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2).setDriveCorrectionSpeed(0.04));
        stones.add(new MasqWayPoint().setPoint(2, 29.5, -90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2).setDriveCorrectionSpeed(0.04));

        stones.add(new MasqWayPoint().setPoint(9, 29.5,-90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2).setDriveCorrectionSpeed(0.04));
        stones.add(new MasqWayPoint().setPoint(15, 29.5, -90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2).setDriveCorrectionSpeed(0.04));
        stones.add(new MasqWayPoint().setPoint(23, 29.5, -90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2).setDriveCorrectionSpeed(0.04));

        while (!opModeIsActive()) {
            position = CVInterpreter.getBlue(robot.cv.detector);
            dash.create("Skystone Position: " + position);
            dash.update();
        }

        waitForStart();

        robot.sideGrabber.rightDown(0);
        robot.sideGrabber.leftUp(0);
        robot.sideGrabber.rightOpen(0);
        robot.sideGrabber.leftClose(0);
        robot.foundationHook.raise();

        if (position == LEFT) runSimultaneously(
                () -> mainAuto(stones.get(1), stones.get(4), stones.get(3), stones.get(5)),
                () -> robot.cv.stop()
        );
        else if (position == MIDDLE) runSimultaneously(
                () -> mainAuto(stones.get(2), stones.get(5), stones.get(3), stones.get(4)),
                () -> robot.cv.stop()
        );
        else runSimultaneously(
                    () -> mainAuto(stones.get(3), stones.get(6), stones.get(2), stones.get(4)),
                    () -> robot.cv.stop()
        );
    }

    private void mainAuto(MasqWayPoint... stones) {
        grabStone(stones[0].setSwitchMode(MECH).setOnComplete(
                () -> {
                    robot.sideGrabber.rightClose(1);
                    robot.sideGrabber.rightMid(0.5);
                }),
                foundationOne,true
        );
        robot.tracker.setDrift(0, -3);
        grabStone(stones[1], foundationThree,false);
        robot.tracker.setDrift(0, -6);
        grabStone(stones[2], foundationTwo,false);
        robot.tracker.setDrift(0, -9);
        grabStone(stones[3], foundationFour, false);
        robot.xyPath(2, park);
    }

    private void grabStone(MasqWayPoint stone, MasqWayPoint foundation, boolean firstStone) {
        if (firstStone) robot.xyPath(4, stone);
        else robot.xyPath(9, bridge2, bridge1.setOnComplete(() -> {
            robot.sideGrabber.rightOpen(0);
            robot.sideGrabber.rightDown(0);
        }), stone.setOnComplete(() -> {
            MasqClock clock = new MasqClock();
            double closeSleep = 1, rotateSleep = 1;
            while(!clock.elapsedTime(closeSleep, SECONDS)) {
                robot.sideGrabber.rightClose(0);
                robot.driveTrain.setVelocity(0);
            }
            clock.reset();
            while(!clock.elapsedTime(rotateSleep, SECONDS)) {
                robot.sideGrabber.rightMid(0);
                robot.driveTrain.setVelocity(0);
            }
            robot.driveTrain.setPower(0);
        }));
        robot.driveTrain.setVelocity(0);
        robot.xyPath(5, bridge1.setOnComplete(null), bridge2, foundation);
        robot.driveTrain.setVelocity(0);
    }
}