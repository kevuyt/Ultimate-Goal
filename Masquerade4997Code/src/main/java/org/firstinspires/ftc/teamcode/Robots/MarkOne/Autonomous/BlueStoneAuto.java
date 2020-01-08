package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;
import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqResources.MasqUtils.velocityAutoController;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.LEFT;
import static org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter.SkystonePosition.MIDDLE;

/**
 * Created by Keval Kataria on 1/4/2020
 */
@Autonomous(name = "Blue Stone", group = "MarkOne")
public class BlueStoneAuto extends MasqLinearOpMode{
    private MarkOne robot = new MarkOne();
    private CVInterpreter.SkystonePosition position;
    private MasqClock timeoutClock;
    private MasqPoint stone1 = new MasqPoint(-17,24.5,90),
            bridge = new MasqPoint(-25,15,90),
            foundation = new MasqPoint(-93,25.5,90),
            stone4 = new MasqPoint(7,26.5,90), rotation = new MasqPoint(-78,18,-90);
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        robot.cv.start();

        while(!opModeIsActive()) {
            position =  CVInterpreter.getPosition(robot.cv.detector);
            dash.create("Position: ", position);
            dash.update();
        }

        waitForStart();
        timeoutClock = new MasqClock();
        runSimultaneously(() -> {robot.cv.stop();},() -> {chooseAuto();});

    }
    private void chooseAuto() {
        if (position == LEFT) runStoneLeft();
        else if (position == MIDDLE) runStoneMiddle();
        else runStoneRight();
    }
    private void runStoneLeft() {
        robot.sideGrabber.rightSlightClose();
        robot.gotoXY(stone1);
        runSimultaneously(() -> {robot.sideGrabber.rightDown();
                    sleep(1.25);
                    robot.sideGrabber.rightClose();
                    sleep(1.25);
                    robot.sideGrabber.rightMid();
                    robot.sideGrabber.leftClose();},
                () -> robot.stopDriving(0.1));
        sleep(1.25);
        robot.gotoXY(bridge,0.75);
        velocityAutoController.setKp(0.006);
        robot.gotoXY(foundation,3.5);
        velocityAutoController.setKp(0.004);
        runSimultaneously(() -> {robot.sideGrabber.rightDown();
            sleep(1);
            robot.sideGrabber.rightSlightClose();
            sleep(1);
            robot.sideGrabber.rightMid();},() -> robot.stopDriving(0.1));
        robot.gotoXY(bridge);
        robot.gotoXY(stone4,4);
        runSimultaneously(() -> {robot.sideGrabber.rightDown();
            sleep(1.25);
            robot.sideGrabber.rightClose();
            sleep(1.25);
            robot.sideGrabber.rightMid();},() -> robot.stopDriving(0.25));/*
        robot.gotoXY(bridge,0.8);
        robot.gotoXY(foundation,3.25);
        robot.sideGrabber.rightDown();
        sleep(1);
        robot.sideGrabber.rightSlightClose();
        sleep(1);
        robot.sideGrabber.rightMid();
        robot.turnAbsolute(180);
        robot.drive(10, Direction.BACKWARD,0.5);
        sleep(0.75);
        robot.foundationHook.lower();
        sleep(1);
        robot.gotoXY(rotation,3);
        robot.foundationHook.raise();
        sleep(1);
        robot.gotoXY(bridge,0.5);*/
        dash.create("Time: ", timeoutClock.seconds());
        dash.update();
        while(!timeoutClock.elapsedTime(29, MasqClock.Resolution.SECONDS)) {sleep();}
    }
    private void runStoneMiddle() {}
    private void runStoneRight() {}
}