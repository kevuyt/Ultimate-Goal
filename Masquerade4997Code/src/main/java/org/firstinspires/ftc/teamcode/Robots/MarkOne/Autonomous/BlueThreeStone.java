package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;
import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint.PointMode.MECH;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.LEFT;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.MIDDLE;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "Blue Three Stone", group = "MarkOne")
public class BlueThreeStone extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    private SkystonePosition position;
    private List<MasqWayPoint> stones = new ArrayList<>();
    private MasqWayPoint
            bridge1 = new MasqWayPoint().setPoint(-18, 18, -90).setSwitchMode(MECH),
            bridge2 = new MasqWayPoint().setPoint(-59, 22.5, -90),
            foundation = new MasqWayPoint().setPoint(-92, 30, -90).setTargetRadius(3);

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        robot.initCamera(hardwareMap);

        stones.add(null);

        stones.add(new MasqWayPoint().setPoint(-19, 29.5, -90).setMinVelocity(0).setTargetRadius(0.5));
        stones.add(new MasqWayPoint().setPoint(-11, 31.5, -90).setMinVelocity(0).setTargetRadius(0.5));
        stones.add(new MasqWayPoint().setPoint(0, 30, -90).setMinVelocity(0).setTargetRadius(0.5));

        stones.add(new MasqWayPoint().setPoint(5, 32.25,-90).setMinVelocity(0).setTargetRadius(0.5));
        stones.add(new MasqWayPoint().setPoint(15, 30, -90).setMinVelocity(0).setTargetRadius(0.5));
        stones.add(new MasqWayPoint().setPoint(22, 28.5, -90).setMinVelocity(0).setTargetRadius(0.5));


        while (!opModeIsActive()) {
            position = CVInterpreter.getPosition(robot.cv.detector);
            dash.create("Skystone Position: ", position);
            dash.update();
        }

        waitForStart();

        robot.sideGrabber.rightRotater.setPosition(0.6);
        robot.sideGrabber.leftUp(0);
        robot.sideGrabber.rightOpen(0);
        robot.sideGrabber.leftClose(0);
        robot.foundationHook.mid();


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
        grabStone(stone1, true);
        grabStone(stone2,false);
        grabStone(stone3, false);
        foundationPark();
    }

    private void grabStone(MasqWayPoint stone, boolean firstStone) {
        if (firstStone) robot.xyPath(4,stone);
        else robot.xyPath(9,bridge2.setSwitchMode(MECH),bridge1, stone);
        robot.driveTrain.stopDriving();
        if (firstStone) robot.sideGrabber.rightDown(0.35);
        else robot.sideGrabber.rightDown(1);
        robot.sideGrabber.rightClose(1);
        robot.sideGrabber.rightMid(1);
        if (firstStone) robot.xyPath(5, bridge1, bridge2.setSwitchMode(MECH), foundation);
        else robot.xyPath(5, bridge1, bridge2, foundation);
        robot.driveTrain.stopDriving();
        robot.sideGrabber.rightSlightClose(0);
        robot.sideGrabber.rightLowMid(0);
    }

    private void foundationPark() {
        sleep(0.25);
        robot.turnAbsolute(179,1.5);
        robot.xyPath(2, new MasqWayPoint().setPoint(robot.tracker.getGlobalX(),
                robot.tracker.getGlobalY()+5,robot.tracker.getHeading()).setMinVelocity(0).setTimeout(1));
        robot.foundationHook.lower();
        sleep();
        MasqUtils.xySpeedController.setKp(0.08);
        MasqUtils.xyAngleController.setKp(0.06);
        MasqWayPoint p1 = new MasqWayPoint().setPoint(new MasqPoint(-80, -10, 30)).setMinVelocity(0.9)
                .setTargetRadius(0.5).setModeSwitchRadius(5);
        MasqWayPoint p2 = new MasqWayPoint().setPoint(new MasqPoint(-80, -5, 0)).setMinVelocity(0.9);
        robot.xyPath(2.5, p1, p2);
        MasqUtils.xySpeedController.setKp(0.045);
        MasqWayPoint park = new MasqWayPoint().setPoint(-45,22,90);
        robot.foundationHook.raise();
        sleep();
        robot.xyPath(2, park);
    }
}