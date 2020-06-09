package org.firstinspires.ftc.teamcode.MarkOne.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MarkOne.Autonomous.Vision.CVInterpreter;
import org.firstinspires.ftc.teamcode.MarkOne.Robot.MarkOne;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint.PointMode.MECH;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "RedIndependent", group = "MarkOne")
public class RedIndependent extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    private CVInterpreter.SkystonePosition position;
    private int stoneCount = 1, maxStones = 3;
    private List<MasqWayPoint> stones = new ArrayList<>();
    private MasqWayPoint
            bridge1Entry = new MasqWayPoint().setPoint(24, 20, 90).setSwitchMode(MECH).setDriveCorrectionSpeed(0.1),
            bridge1Exit = new MasqWayPoint().setPoint(24, 22, 90).setSwitchMode(MECH).setDriveCorrectionSpeed(0.1),
            bridge2 = new MasqWayPoint().setPoint(60, 25, 90).setSwitchMode(MECH).setOnComplete(() -> {
                robot.sideGrabber.leftClose(0);
                robot.sideGrabber.leftUp(0);
            }),
            park = new MasqWayPoint().setPoint(35,24, 180).setMinVelocity(0),
            foundationOne = new MasqWayPoint().setPoint(82, 32, 90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.leftSlightClose(0);
                robot.sideGrabber.leftLowMid(0);
            }),
            foundationTwo = new MasqWayPoint().setPoint(88, 32, 90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.leftSlightClose(0);
                robot.sideGrabber.leftLowMid(0);
            }),
            foundationThree = new MasqWayPoint().setPoint(94, 32, 90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
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
        stones.add(new MasqWayPoint().setPoint(12, 28, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(4, 28, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(-4, 28, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));

        stones.add(new MasqWayPoint().setPoint(-13, 28,90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(-21, 28, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(-29, 28, 90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2).setDriveCorrectionSpeed(0.04));

        while (!opModeIsActive()) {
            position = CVInterpreter.getRed(robot.camera.detector);
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

        if (position == CVInterpreter.SkystonePosition.RIGHT) runStones = rightStones();
        else if (position == CVInterpreter.SkystonePosition.MIDDLE) runStones = middleStones();
        else runStones = leftStones();

        runSimultaneously (
                () -> mainAuto(runStones),
                () -> robot.camera.stop()
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
        robot.tracker.setDrift(-3, 0);
        grabStone(stones[0], foundationThree);
        robot.tracker.setDrift(-3, 2);
        grabStone(stones[1], foundationTwo);
        robot.tracker.setDrift(-3, 4);
        bridge1Exit = bridge1Exit.setX(bridge1Exit.getX() + 10);
        grabStone(stones[2], foundationOne);
        foundationPark();
    }

    private void grabStone(MasqWayPoint stone, MasqWayPoint foundation) {
        if (stoneCount == 1) robot.xyPath(4, stone);
        else robot.xyPath(9, bridge2, bridge1Exit.setOnComplete(() -> {
            robot.sideGrabber.leftOpen(0);
            robot.sideGrabber.leftDown(0);
        }), stone);
        robot.xyPath(5, exitStone(), bridge1Entry, bridge2, foundation);
        robot.driveTrain.setVelocity(0);
        stoneCount++;
    }

    private void foundationPark() {
        initFoundationControllers();
        robot.sideGrabber.leftUp(0);
        robot.sideGrabber.leftOpen(0);
        robot.turnAbsolute(-175,1);
        robot.drive(7, Direction.BACKWARD, 1);
        robot.foundationHook.lower();
        sleep();
        MasqWayPoint moveFoundation = new MasqWayPoint().setPoint(robot.tracker.getGlobalX(),
                -5, 178).setAngularCorrectionSpeed(0.04).setSwitchMode(MECH).setDriveCorrectionSpeed(0.3);

        robot.xyPath(2, moveFoundation);

        runSimultaneously(
                () -> robot.foundationHook.raise(),
                () -> robot.stop(1)
        );
        MasqWayPoint p2 = new MasqWayPoint().setPoint(80,0, -robot.tracker.getHeading()).setMinVelocity(0).setSwitchMode(MECH);
        robot.xyPath(6, p2, park);
        robot.stop(0.5);
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
        };
    }
    private MasqWayPoint[] leftStones() {
        return new MasqWayPoint[] {
                stones.get(3),
                stones.get(6),
                stones.get(2),
        };
    }
    private MasqWayPoint[] rightStones() {
        return new MasqWayPoint[] {
                stones.get(1),
                stones.get(4),
                stones.get(3),
        };
    }

    private void initFoundationControllers() {
        MasqUtils.driveController = new MasqPIDController(0.005);
        MasqUtils.angleController = new MasqPIDController(0.003);
        MasqUtils.turnController = new MasqPIDController(0.01);
        MasqUtils.velocityTeleController = new MasqPIDController(0.001);
        MasqUtils.velocityAutoController = new MasqPIDController(0.0045);
        MasqUtils.xySpeedController = new MasqPIDController(0.045, 0, 0);
        MasqUtils.xyAngleController = new MasqPIDController(0.04, 0, 0);
    }
}