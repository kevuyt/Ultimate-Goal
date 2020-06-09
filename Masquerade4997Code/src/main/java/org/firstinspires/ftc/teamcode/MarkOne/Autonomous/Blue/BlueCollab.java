package org.firstinspires.ftc.teamcode.MarkOne.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MarkOne.Autonomous.Vision.CVInterpreter;
import org.firstinspires.ftc.teamcode.MarkOne.Robot.MarkOne;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint.PointMode.MECH;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "BlueCollab", group = "MarkOne")
public class BlueCollab extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    private CVInterpreter.SkystonePosition position;
    private int stoneCount = 1, maxStones = 4, reducedXPBasedYCorrection = 2;
    private List<MasqWayPoint> stones = new ArrayList<>();
    private MasqWayPoint
            bridge1Entry = new MasqWayPoint().setPoint(-24, 20, -90).setSwitchMode(MECH).setDriveCorrectionSpeed(0.1),
            bridge1Exit = new MasqWayPoint().setPoint(-24, 22, -90).setSwitchMode(MECH).setDriveCorrectionSpeed(0.1),
            bridge2 = new MasqWayPoint().setPoint(-60, 22, -90).setSwitchMode(MECH).setDriveCorrectionSpeed(0.1)
                    .setOnComplete(() -> {
                        robot.sideGrabber.rightClose(0);
                        robot.sideGrabber.rightUp(0);
                    }
            ),
            park = new MasqWayPoint().setPoint(-35,24, 180).setMaxVelocity(1).setMinVelocity(0),
            foundationOne = new MasqWayPoint().setPoint(-86, 30, -90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.rightSlightClose(0);
                robot.sideGrabber.rightLowMid(0);
            }),
            foundationTwo = new MasqWayPoint().setPoint(-88, 30, -90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.rightSlightClose(0);
                robot.sideGrabber.rightLowMid(0);
            }),
            foundationThree = new MasqWayPoint().setPoint(-90, 30, -90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.rightSlightClose(0);
                robot.sideGrabber.rightLowMid(0);
            }),
            foundationFour = new MasqWayPoint().setPoint(-45, 20, -180).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.rightLowMid(0);
                robot.sideGrabber.rightOpen(0);
            });


    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        robot.initCamera(hardwareMap);

        MasqUtils.velocityAutoController = new MasqPIDController(0.003);

        stones.add(null);

        stones.add(new MasqWayPoint().setPoint(-12, 28, -90));
        stones.add(new MasqWayPoint().setPoint(-4, 28, -90));
        stones.add(new MasqWayPoint().setPoint(4, 28, -90));

        stones.add(new MasqWayPoint().setPoint(13, 28,-90));
        stones.add(new MasqWayPoint().setPoint(21, 28, -90));
        stones.add(new MasqWayPoint().setPoint(29, 28, -90));

        while (!opModeIsActive()) {
            position = CVInterpreter.getBlue(robot.camera.detector);
            dash.create("Skystone Position: " + position);
            dash.update();
        }

        waitForStart();

        timeoutClock.reset();
        robot.sideGrabber.rightDown(0);
        robot.sideGrabber.leftUp(0);
        robot.sideGrabber.rightOpen(0);
        robot.sideGrabber.leftClose(0);
        robot.foundationHook.raise();
        MasqWayPoint[] runStones;

        if (position == CVInterpreter.SkystonePosition.RIGHT) runStones = rightStones();
        else if (position == CVInterpreter.SkystonePosition.MIDDLE) runStones = middleStones();
        else runStones = leftStones();

        runSimultaneously(
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
                            robot.sideGrabber.rightClose(closeSleep);
                            robot.sideGrabber.rightUp(upSleep);
                        }
                );
            }).setSwitchMode(MECH).setMinVelocity(0);
            index++;
        }
        robot.tracker.setDrift(4, 0);
        grabStone(stones[0], foundationOne.setY(foundationOne.getY() + reducedXPBasedYCorrection));
        robot.tracker.setDrift(4, -3);
        grabStone(stones[1], foundationTwo.setY(foundationTwo.getY() + reducedXPBasedYCorrection));
        robot.tracker.setDrift(4, -6);
        grabStone(stones[2], foundationThree.setY(foundationThree.getY() + reducedXPBasedYCorrection));
        robot.tracker.setDrift(4, -8);
        grabStone(stones[3], foundationFour.setY(foundationFour.getY() + reducedXPBasedYCorrection));
        park();
    }

    private void grabStone(MasqWayPoint stone, MasqWayPoint foundation) {
        if (stoneCount == 1) robot.xyPath(4, stone);
        else robot.xyPath(9, bridge2, bridge1Exit.setOnComplete(() -> {
            robot.sideGrabber.rightOpen(0);
            robot.sideGrabber.rightDown(0);
        }), stone);
        robot.driveTrain.setVelocity(0);
        if (stoneCount == 4) {
            bridge2.setH(180).setOnComplete(() -> {
                robot.sideGrabber.leftDown(0);
                robot.sideGrabber.leftOpen(0);
            });
        }
        robot.xyPath(5, exitStone(), bridge1Entry, bridge2, foundation);
        robot.driveTrain.setVelocity(0);
        stoneCount++;
    }

    private void park() {
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
                stones.get(5).setDriveCorrectionSpeed(0.04).setY(30),
                stones.get(3).setX(stones.get(3).getX() + 2).setY(32).setTimeout(1.5),
                stones.get(1).setX(stones.get(1).getX() + 2).setY(34).setDriveCorrectionSpeed(0.04).setTimeout(1.5)
        };
    }
    private MasqWayPoint[] rightStones() {
        return new MasqWayPoint[] {
                stones.get(3),
                stones.get(6).setDriveCorrectionSpeed(0.04).setY(30),
                stones.get(2).setX(stones.get(2).getX() + 2).setY(32).setTimeout(1),
                stones.get(1).setX(stones.get(1).getX() + 2).setY(34).setDriveCorrectionSpeed(0.05).setTimeout(1)
        };
    }
    private MasqWayPoint[] leftStones() {
        return new MasqWayPoint[]{
                stones.get(1),
                stones.get(4).setDriveCorrectionSpeed(0.04).setY(30),
                stones.get(3).setX(stones.get(3).getX() + 2).setY(32).setTimeout(1),
                stones.get(2).setX(stones.get(2).getX() + 2).setY(34).setDriveCorrectionSpeed(0.05).setTimeout(1)
        };
    }
}