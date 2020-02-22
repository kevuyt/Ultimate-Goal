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
import static Library4997.MasqResources.MasqHelpers.Direction.BACKWARD;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.LEFT;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.MIDDLE;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "Blue Collab", group = "MarkOne")
public class BlueCollabAuto extends MasqLinearOpMode {
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
            park = new MasqWayPoint().setPoint(-45,28, 90)
            .setDriveCorrectionSpeed(0.2).setLookAhead(5);

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        robot.initCamera(hardwareMap);

        stones.add(null);

        stones.add(new MasqWayPoint().setPoint(-17.5, 29.5, -90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(-8.5, 29.5, -90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(2, 29.5, -90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));

        stones.add(new MasqWayPoint().setPoint(9, 29.5,-90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(15, 29.5, -90).setMinVelocity(0).setTargetRadius(0.5).setModeSwitchRadius(2));
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

    private void mainAuto(MasqWayPoint stone1, MasqWayPoint stone2, MasqWayPoint stone3, MasqWayPoint stone4) {
        grabStone(stone1.setSwitchMode(MECH).setOnComplete(() -> {
            robot.sideGrabber.rightClose(1);
            robot.sideGrabber.rightMid(0.5);
        }), foundationOne,true
        );
        grabStone (
                stone2.setPoint(stone2.getX(), stone2.getY() + 3, stone2.getH()),
                foundationThree,false
        );
        grabStone (
                stone3.setPoint(stone3.getX(), stone3.getY() + 6, stone3.getH()),
                foundationTwo,false
        );
        grabStone(
                stone4.setPoint(stone4.getX(), stone4.getY() + 9, stone4.getH()),
                foundationFour, false
        );
        robot.xyPath(2, park);
    }

    private void grabStone(MasqWayPoint stone, MasqWayPoint foundation, boolean firstStone) {
        if (firstStone) robot.xyPath(4, stone);
        else robot.xyPath(9, bridge2, bridge1.setY(bridge1.getY() + 3).setOnComplete(() -> {
            robot.sideGrabber.rightOpen(0);
            robot.sideGrabber.rightDown(0);
        }), stone.setOnComplete(() -> {
            double closeSleep = 1, rotateSleep = 1;
            //robot.stop(closeSleep + rotateSleep);
            robot.sideGrabber.rightClose(closeSleep);
            robot.sideGrabber.rightMid(rotateSleep);
        }));
        robot.driveTrain.setVelocity(0);
        robot.xyPath(5, bridge1.setOnComplete(null), bridge2, foundation);
        robot.driveTrain.setVelocity(0);
    }

    private void foundationPark() {
        robot.turnAbsolute(178,1);
        robot.drive(5,1,BACKWARD,1);
        robot.foundationHook.lower();
        sleep();
        MasqWayPoint p1 = new MasqWayPoint().setPoint(-60,0, 90)
                .setDriveCorrectionSpeed(0.2).setMinVelocity(0);
        MasqWayPoint p2 = new MasqWayPoint().setPoint(-90,20, 60)
                .setDriveCorrectionSpeed(1).setTimeout(1).setSwitchMode(MECH).setOnSimul(() -> robot.foundationHook.raise());

        robot.xyPath(5, p1, p2, park);
        robot.stop(0.5);
    }

    private void foundationRotation(double inches, double heading) {
        double curr = Math.abs(robot.driveTrain.leftDrive.getInches());
        double tar = curr + inches;
        MasqClock timeout = new MasqClock();
        while (curr < tar && opModeIsActive() &&
                !timeout.elapsedTime(2, MasqClock.Resolution.SECONDS)) {
            robot.driveTrain.setVelocity(0.5, 1);
            robot.tracker.updateSystem();
            curr = Math.abs(robot.driveTrain.leftDrive.getInches());
        }
        robot.turnAbsolute(-90, 1);
        robot.driveTrain.setVelocity(0);
        robot.foundationHook.raise();
        robot.stop(1);
    }
}