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

import static Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint.PointMode.MECH;
import static Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint.PointMode.SWITCH;
import static Library4997.MasqResources.MasqHelpers.Direction.BACKWARD;
import static Library4997.MasqResources.MasqUtils.DEFAULT_SLEEP_TIME;
import static Library4997.MasqResources.MasqUtils.DEFAULT_TIMEOUT;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.LEFT;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.MIDDLE;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "Blue", group = "MarkOne")
public class BlueThreeStone extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    private SkystonePosition position;
    private List<MasqWayPoint> stones = new ArrayList<>();
    private MasqWayPoint
            bridge1 = new MasqWayPoint().setPoint(-24, 20, -90).setSwitchMode(MECH),
            bridge2 = new MasqWayPoint().setPoint(-59, 24, -90).setSwitchMode(MECH),
            foundationOne = new MasqWayPoint().setPoint(-84, 30, -90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.rightSlightClose(0);
                robot.sideGrabber.rightLowMid(0);
            }),
            foundationTwo = new MasqWayPoint().setPoint(-87, 30, -90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.rightSlightClose(0);
                robot.sideGrabber.rightLowMid(0);
            }),
            foundationThree = new MasqWayPoint().setPoint(-92, 30, -90).setTargetRadius(3).setMinVelocity(0).setOnComplete(() -> {
                robot.sideGrabber.rightSlightClose(0);
                robot.sideGrabber.rightLowMid(0);
            });

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        robot.initCamera(hardwareMap);

        stones.add(null);

        stones.add(new MasqWayPoint().setPoint(-17.5, 29.5, -90).setMinVelocity(0).setTargetRadius(0.5).setSwitchMode(SWITCH).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(-8.5, 29.5, -90).setMinVelocity(0).setTargetRadius(0.5).setSwitchMode(SWITCH).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(-3, 29.5, -90).setMinVelocity(0).setTargetRadius(0.5).setSwitchMode(SWITCH).setModeSwitchRadius(2));

        stones.add(new MasqWayPoint().setPoint(7, 29.5,-90).setMinVelocity(0).setTargetRadius(0.5).setSwitchMode(SWITCH).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(15, 29.5, -90).setMinVelocity(0).setTargetRadius(0.5).setSwitchMode(SWITCH).setModeSwitchRadius(2));
        stones.add(new MasqWayPoint().setPoint(22, 29.5, -90).setMinVelocity(0).setTargetRadius(0.5).setSwitchMode(SWITCH).setModeSwitchRadius(2));

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

        //mainAuto(stones.get(1), stones.get(4),stones.get(2));

        if (position == LEFT) runSimultaneously(
                () -> mainAuto(stones.get(1), stones.get(4),stones.get(2)),
                () -> robot.cv.stop()
        );
        else if (position == MIDDLE) runSimultaneously(
                () -> mainAuto(stones.get(2), stones.get(5),stones.get(1)),
                () -> robot.cv.stop()
        );
        else runSimultaneously(
                () -> mainAuto(stones.get(3), stones.get(6),stones.get(1)),
                () -> robot.cv.stop()
        );
    }

    private void mainAuto(MasqWayPoint stone1, MasqWayPoint stone2, MasqWayPoint stone3) {
        grabStone(stone1.setSwitchMode(MECH).setOnComplete(() -> {
            robot.sideGrabber.rightDown(0.35);
            robot.sideGrabber.rightClose(1);
            robot.sideGrabber.rightMid(0.5);
        }), foundationOne,true);
        grabStone(stone2, foundationThree,false);
        grabStone(stone3.setPoint(stone3.getX(), stone3.getY()+3, stone3.getH()), foundationTwo,false);
        foundationPark();
    }

    private void grabStone(MasqWayPoint stone, MasqWayPoint foundation, boolean firstStone) {
        if (firstStone) robot.xyPath(4, stone);
        else robot.xyPath(9, bridge2, bridge1.setOnComplete(() -> {
            robot.sideGrabber.rightOpen(0);
            robot.sideGrabber.rightDown(0);
        }), stone.setOnComplete(() -> {
            robot.sideGrabber.rightClose(1);
            robot.sideGrabber.rightMid(1);
        }));
        robot.driveTrain.setVelocity(0);
        //if (firstStone) robot.sideGrabber.rightDown(0.35);
        /*else robot.sideGrabber.rightDown(1);
        robot.sideGrabber.rightClose(1);
        robot.sideGrabber.rightMid(1);*/
        robot.xyPath(5, bridge1.setOnComplete(null), bridge2, foundation);
        robot.driveTrain.setVelocity(0);
        /*robot.sideGrabber.rightOpen(0);
        robot.sideGrabber.rightLowMid(0);*/
    }

    private void foundationPark() {
        robot.turnAbsolute(180,1.5);
        robot.drive(7, BACKWARD);
        robot.foundationHook.lower();
        sleep();
        robot.driveTrain.setVelocity(1);
        sleep(1.);
        robot.driveTrain.setVelocity(0);
        robot.turnAbsolute(-70,1);
        robot.foundationHook.raise();
        robot.stop(1);
        MasqWayPoint park = new MasqWayPoint().setPoint(-45,28,90).setDriveCorrectionSpeed(0.2);
        robot.xyPath(1,park);
        robot.stop(0.2);
    }
}