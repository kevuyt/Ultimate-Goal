package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "Blue Full", group = "MarkOne")
public class BlueThreeStone extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    private SkystonePosition position;
    private List<MasqPoint> stones = new ArrayList<>();
    private MasqWayPoint
            bridge1 = new MasqWayPoint().setPoint(-25, 19, -90).setMinVelocity(0.9).setTargetRadius(5),
            bridge2 = new MasqWayPoint().setPoint(-59, 20, -90).setTargetRadius(3).setMinVelocity(0.7),
            foundation = new MasqWayPoint().setPoint(-90, 28, -90).setTargetRadius(3).setMinVelocity(0.4);

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();

        stones.add(null);

        stones.add(new MasqPoint(-17, 30, 90));
        stones.add(new MasqPoint(-7, 29, 90));
        stones.add(new MasqPoint(0, 30, 90));

        stones.add(new MasqPoint(7.5, 28, 90));
        stones.add(new MasqPoint(15, 30, 90));
        stones.add(new MasqPoint(22, 28.5, 90));

        //robot.cv.start();

        while (!opModeIsActive()) {
            //position = CVInterpreter.getPosition(robot.cv.detector);
            dash.create("Skystone Position: ", position);
            dash.update();
        }

        waitForStart();

        robot.sideGrabber.rightUp(0);
        robot.sideGrabber.leftUp(0);
        robot.sideGrabber.rightClose(0);
        robot.sideGrabber.leftClose(0);


        /*if (position == LEFT) runSimultaneously(
                () -> mainAuto(stones.get(1), stones.get(4)),
                () -> robot.cv.stop()
        );
        else if (position == MIDDLE) runSimultaneously(
                () -> mainAuto(stones.get(2), stones.get(5)),
                () -> robot.cv.stop()
        );
        else runSimultaneously(
                () -> mainAuto(stones.get(3), stones.get(6)),
                () -> robot.cv.stop()
        );*/
    }

    private void mainAuto(MasqWayPoint stone1, MasqWayPoint stone2, MasqWayPoint stone3) {
        foundationPark();
    }

    private void grabStone(MasqWayPoint stone) {
        robot.sideGrabber.rightDown(0);
        robot.sideGrabber.rightClose(0);
        robot.sideGrabber.rightMid(0);
        robot.xyPath(2,stone);
        //servo stuff
        robot.xyPath(3, bridge1, bridge2, foundation);
        //servo stuff
    }

    private void foundationPark() {
        MasqWayPoint p1 = new MasqWayPoint().setPoint(new MasqPoint(-68, 5, 90));
        MasqWayPoint p2 = new MasqWayPoint().setPoint(new MasqPoint(-75, 5, 90));
        robot.xyPath(4, p1, p2);
        MasqWayPoint park = new MasqWayPoint();
        robot.foundationHook.raise();
        robot.xyPath(4, park);
        dash.create("Completed Path");
        dash.update();
        sleep(10);
    }
}