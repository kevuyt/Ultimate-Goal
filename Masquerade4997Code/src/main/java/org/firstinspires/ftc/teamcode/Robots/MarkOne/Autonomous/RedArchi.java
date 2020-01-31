package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;
import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint.PointMode.MECH;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.RIGHT;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "RedArchi", group = "MarkOne")
public class RedArchi extends MasqLinearOpMode{
    private MarkOne robot = new MarkOne();
    private CVInterpreter.SkystonePosition position;
    private List<MasqWayPoint> stones = new ArrayList<>();
    private MasqWayPoint
            bridge1 = new MasqWayPoint().setPoint(18,18,90).setSwitchMode(MECH),
            bridge2 = new MasqWayPoint().setPoint(41,22.5,90),
            foundationOne = new MasqWayPoint().setPoint(65, 30, 90).setTargetRadius(3).setMinVelocity(0),
            foundationTwo = new MasqWayPoint().setPoint(70, 30, 90).setTargetRadius(3).setMinVelocity(0),
            foundationThree = new MasqWayPoint().setPoint(75, 30, 90).setTargetRadius(3).setMinVelocity(0);
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        robot.initCamera(hardwareMap);

        double stonePosOne = 8;

        stones.add(new MasqWayPoint().setPoint(stonePosOne,28,90).setMinVelocity(0).setTargetRadius(0.5).setTimeout(4));
        stones.add(new MasqWayPoint().setPoint(stonePosOne - 8,28,90).setMinVelocity(0).setTargetRadius(0.5));
        stones.add(new MasqWayPoint().setPoint(stonePosOne - (8 * 2),28,90).setMinVelocity(0).setTargetRadius(0.5));

        stones.add(new MasqWayPoint().setPoint(stonePosOne - (8 * 3),28,90).setMinVelocity(0).setTargetRadius(0.5));
        stones.add(new MasqWayPoint().setPoint(stonePosOne - (8 * 4),28,90).setMinVelocity(0).setTargetRadius(0.5));
        stones.add(new MasqWayPoint().setPoint(stonePosOne - (8 * 5),28,90).setMinVelocity(0).setTargetRadius(0.5));

        while(!opModeIsActive()) {
            position = CVInterpreter.getPosition(robot.cv.detector);
            dash.create("Skystone Position: ", position);
            dash.update();
        }

        waitForStart();

        robot.sideGrabber.leftRotater.setPosition(0.6);
        robot.sideGrabber.rightUp(0);
        robot.sideGrabber.leftOpen(0);
        robot.sideGrabber.rightClose(0);
        robot.foundationHook.mid();

        if (position == RIGHT) runSimultaneously(
                () -> mainAuto(stones.get(1), stones.get(4),stones.get(2)),
                () -> robot.cv.stop()
        );
        else if (position == MIDDLE) runSimultaneously(
                () -> mainAuto(stones.get(2), stones.get(5),stones.get(1)),
                () -> robot.cv.stop()
        );
        else runSimultaneously(
                    () -> mainAuto(stones.get(3), stones.get(6), stones.get(1)),
                    () -> robot.cv.stop()
            );
    }
    private void mainAuto(MasqWayPoint stone1, MasqWayPoint stone2, MasqWayPoint stone3) {
        grabStone(stone1, foundationOne,true);
        grabStone(stone2, foundationTwo,false);
        grabStone(stone3, foundationThree,false);
        foundationPark();
    }

    private void grabStone(MasqWayPoint stone, MasqWayPoint foundation, boolean firstStone) {
        MasqUtils.xyAngleController = new MasqPIDController(0.06, 0, 0);
        if (firstStone) robot.xyPath(4, stone);
        else robot.xyPath(9, bridge2.setSwitchMode(MECH), bridge1, stone);
        robot.driveTrain.stopDriving();
        if (firstStone) robot.sideGrabber.leftDown(0.35);
        else robot.sideGrabber.leftDown(1);
        robot.sideGrabber.leftClose(1);
        robot.sideGrabber.leftMid(1);
        if (firstStone) robot.xyPath(5, bridge1, bridge2.setSwitchMode(MECH), foundation);
        else robot.xyPath(5, bridge1, bridge2, foundation);
        robot.driveTrain.stopDriving();
        robot.sideGrabber.leftSlightClose(0);
        robot.sideGrabber.leftLowMid(0);
    }

    private void foundationPark() {
        robot.turnAbsolute(170,1.5);
        robot.drive(7, Direction.BACKWARD);
        robot.foundationHook.lower();
        sleep();
        MasqWayPoint p1 = new MasqWayPoint()
                .setPoint(new MasqPoint(80, 5, -80))
                .setMinVelocity(0.5)
                .setModeSwitchRadius(5);
        robot.xyPath(3, p1);
        robot.foundationHook.raise();
        sleep();
        MasqWayPoint park = new MasqWayPoint().setPoint(45,22,-90);
        robot.xyPath(2, park);
    }
}