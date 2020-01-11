package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import java.util.ArrayList;
import java.util.List;

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
            rotation = new MasqPoint(-84,28,179);
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        stones.add(null);
        stones.add(new MasqPoint(-17,29,90));
        stones.add(new MasqPoint(-7,24.75,90));
        stones.add(null);
        stones.add(new MasqPoint(7,25.5,90));
        stones.add(null);
        stones.add(null);
        // robot.cv.start();

        while(!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }

        waitForStart();
        robot.resetServos();
        //runSimultaneously(() -> robot.cv.stop(), this::chooseAuto);
        runStoneLeft();
    }
    private void chooseAuto() {
        if (position == LEFT) runStoneLeft();
        else if (position == MIDDLE) runStoneMiddle();
        else runStoneRight();
    }
    private void runStoneLeft() {
        robot.sideGrabber.rightSlightClose();
        robot.gotoXY(stones.get(1));
        robot.sideGrabber.rightDown();
        robot.sideGrabber.rightClose();
        robot.sideGrabber.rightMid();
        robot.sideGrabber.leftClose();
        robot.gotoXY(bridge,1);
        robot.gotoXY(foundation,2.5,3);
        robot.sideGrabber.rightSlightClose();
        robot.gotoXY(bridge,1);
        robot.gotoXY(stones.get(4),4);
        robot.sideGrabber.rightDown();
        robot.sideGrabber.rightClose();
        robot.sideGrabber.rightMid();
        robot.gotoXY(bridge,1);
        robot.gotoXY(foundation,2.5,3);
        robot.sideGrabber.rightSlightClose();
        /*
        robot.gotoXY(bridge,1);
        robot.gotoXY(stones.get(2),2.5);
        robot.sideGrabber.rightDown();
        robot.sideGrabber.rightClose();
        robot.sideGrabber.rightMid();
        robot.gotoXY(bridge,1);
        velocityAutoController.setKp(0.006);
        robot.gotoXY(foundation,2.5);
        velocityAutoController.setKp(0.004);
        robot.sideGrabber.rightSlightClose();*/
        robot.gotoXY(rotation);
        robot.foundationHook.lower();
        sleep();
    }
    private void runStoneMiddle() {}
    private void runStoneRight() {}
}