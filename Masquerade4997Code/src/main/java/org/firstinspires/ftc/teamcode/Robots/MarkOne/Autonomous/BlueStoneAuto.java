package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.LEFT;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.MIDDLE;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "Blue Stone", group = "MarkOne")
public class BlueStoneAuto extends MasqLinearOpMode{
    private MarkOne robot = new MarkOne();
    private SkystonePosition position;
    private MasqClock timeoutClock;
    private List<MasqPoint> stones = new ArrayList<>();
    private MasqPoint
            bridge = new MasqPoint(-25,15,90),
            foundation = new MasqPoint(-88,28,90),
            grabFoundation = new MasqPoint(-88,34,90),
            rotation = new MasqPoint(-84,28,179);
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        stones.add(null);
        stones.add(new MasqPoint(-17,29,90));
        stones.add(new MasqPoint(-7,24.75,90));
        stones.add(null);
        stones.add(new MasqPoint(4,25.5,90));
        stones.add(null);
        stones.add(null);
        // robot.cv.start();

        while(!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }

        waitForStart();
        robot.sideGrabber.rightUp();
        robot.sideGrabber.leftUp();
        robot.sideGrabber.rightOpen();
        robot.sideGrabber.leftClose();
        //runSimultaneously(() -> robot.cv.stop(), this::chooseAuto);
        runStoneLeft();
    }
    private void chooseAuto() {
        if (position == LEFT) runStoneLeft();
        else if (position == MIDDLE) runStoneMiddle();
        else runStoneRight();
    }
    private void runStoneLeft() {
        robot.gotoXY(stones.get(1));
        robot.sideGrabber.rightDown();
        robot.sideGrabber.rightClose();
        robot.sideGrabber.rightUp();
        robot.gotoXY(bridge,1);
        robot.gotoXY(foundation,2.5,3);
        robot.sideGrabber.rightOpen();
        robot.sideGrabber.rightUp();
        robot.gotoXY(bridge,1);
        robot.gotoXY(stones.get(4),4);
        robot.sideGrabber.rightDown();
        robot.sideGrabber.rightClose();
        robot.sideGrabber.rightUp();
        robot.gotoXY(bridge,1);
        robot.gotoXY(foundation,2.5,3);
        robot.sideGrabber.rightSlightClose();
        robot.turnRelative(90, Direction.LEFT);
        robot.gotoXY(robot.tracker.getGlobalX(), robot.tracker.getGlobalY() + 6,
                robot.tracker.getHeading(), 2, 0.5, 1);
        robot.foundationHook.lower();
        sleep(1);
        // Need to resolve this thing with the 180.
        robot.gotoXY(new MasqPoint(-74, 0, -90));
    }
    private void runStoneMiddle() {}
    private void runStoneRight() {}
}