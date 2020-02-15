package org.firstinspires.ftc.teamcode.Robots.MarkOne.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 2020-02-15.
 * Project: MasqLib
 */
@TeleOp(name = "Ghost", group = "MarkOne")
public class Ghost extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    private double currVel = 0;
    private List<MasqWayPoint> points = new ArrayList<>();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeTeleop();

        while(!opModeIsActive()) {
            dash.create("Heading: ", robot.tracker.getHeading());
            dash.update();
        }

        waitForStart();
        points.add(robot.getCurrentWayPoint());
        int index = 1;
        while(opModeIsActive()) {
            MasqWayPoint point = new MasqWayPoint();
            teleop();
            point = point.setPoint(robot.tracker.getGlobalX(), robot.tracker.getGlobalY(), robot.tracker.getHeading());
            double dXY = points.get(index - 1).getPoint().distanceToVector(point.getPoint());
            double switchPoint = dXY / 1.5;
            point = point.setPointSwitchRadius(switchPoint).setMinVelocity(currVel).setMaxVelocity(currVel);
            point = point.setOnComplete(
                    () -> {
                        robot.foundationHook.getSet();
                        robot.sideGrabber.getSet();
                    }
            );
            points.add(point);
        }

    }
    private void teleop() {
        if (controller1.rightBumper() || controller1.leftBumper())
            robot.MECH(controller1,0.5, 0.2);
        else robot.MECH(controller1,1, 0.5);

        currVel = robot.driveTrain.getVelocity();

        if (controller1.leftTriggerPressed()) robot.intake.setVelocity(-1);
        else if (controller1.rightTriggerPressed()) robot.intake.setVelocity(1);
        else robot.intake.setVelocity(0);

        robot.lift.setVelocity(0.75 * controller2.leftStickY());

        robot.tapeMeasure.setPower(controller2.rightStickY());

        if (controller2.a()) robot.blockGrabber.setPosition(0);
        else if (controller2.x()) robot.blockGrabber.setPosition(1);
        robot.toggleBlockRotator(controller2);
        robot.toggleCapper(controller2);

        robot.foundationHook.DriverControl(controller1);

        robot.sideGrabber.teleReset();

        robot.tracker.updateSystem();
        dash.update();

        controller1.update();
        controller2.update();
    }



}