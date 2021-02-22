package org.firstinspires.ftc.teamcode.Osiris.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqMath.MasqWayPoint;
import Library4997.MasqResources.MasqLinearOpMode;

import static Library4997.MasqRobot.OpMode.AUTO;
import static Library4997.MasqMath.MasqWayPoint.PointMode.*;
import static org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.ZoneFinder.TargetZone.*;
import static org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.ZoneFinder.*;

/**
 * Created by Keval Kataria on 11/27/2020
 */
@Autonomous(name = "Red", group = "Osiris")
public class Red extends MasqLinearOpMode {
    private Osiris robot = new Osiris();
    private TargetZone zone;
    private MasqWayPoint target = new MasqWayPoint().setTimeout(5).setSwitchMode(SWITCH).setTargetRadius(5).setAngularCorrectionSpeed(0.0035),
            strafe = new MasqWayPoint(-7,-30,0).setSwitchMode(TANK).setAngularCorrectionSpeed(0.002);

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap, AUTO);

        while(!opModeIsActive()) {
            zone = findZone(robot.camera.detector);

            dash.create("Zone: " + zone);
            dash.create("Control: " + robot.detector.getControl());
            dash.create("Top: " + robot.detector.getTop());
            dash.create("Bottom: " + robot.detector.getBottom());
            dash.update();
        }

        waitForStart();

        timeoutClock.reset();
        robot.camera.stop();
        robot.tracker.reset();

        robot.claw.lower();

        if (zone == A) target = target.setPoint(-7,-63.5,70);
        else if (zone == B) target = target.setPoint(-3,-80,-20).setPointSwitchRadius(24);
        else target = target.setPoint(-10,-103,30);


        if(zone != A) robot.xyPath(strafe, target);
        else robot.xyPath(target);
        robot.turnAbsolute(-target.getH(),1);

        robot.shooter.setVelocity(0.52);
        robot.claw.open();
        robot.hopper.setPosition(1);
        sleep();
        robot.claw.raise();

        robot.xyPath(new MasqWayPoint(0,-68,180).setTimeout(20).setDriveCorrectionSpeed(0.035));
        robot.turnAbsolute(180);
        flick(1);

        robot.xyPath(new MasqWayPoint(8, -68, 180).setDriveCorrectionSpeed(0.015));
        robot.turnAbsolute(180);
        flick(1);

        robot.xyPath(new MasqWayPoint(17, -68, 180).setDriveCorrectionSpeed(0.015));
        robot.turnAbsolute(180);
        flick(1);

        robot.shooter.setVelocity(0);

        if(zone != A) {
            robot.turnAbsolute(-45);

            if(zone == C) shootStack(3);
            shootStack(1);
        }

        /**/
        robot.xyPath(new MasqWayPoint(robot.tracker.getGlobalX(), -72, 0).setSwitchMode(SWITCH));
        robot.turnAbsolute(0);

        while(timeoutClock.hasNotPassed(29.5)) {
            robot.tracker.updateSystem();
            dash.create(robot.tracker);
            dash.update();
        }
    }

    private void flick(int iterations) {
        for (int i = 0; i < iterations; i++) {
            robot.flicker.setPosition(0.9);
            sleep(1.0);

            robot.flicker.setPosition(0);
            sleep(1.0);
        }
    }
    private void shootStack(int rings) {
        robot.hopper.setPosition(0);
        robot.intake.setVelocity(-1);
        robot.xyPath(new MasqWayPoint(-6, -44, robot.tracker.getHeading()).setTimeout(4).setSwitchMode(SWITCH).setAngularCorrectionSpeed(0.12));
        robot.shooter.setVelocity(0.6);
        sleep();
        robot.intake.setVelocity(0);
        robot.turnAbsolute(0,4);
        robot.hopper.setPosition(1);
        robot.xyPath(new MasqWayPoint(9, -60, 180).setSwitchMode(SWITCH).setModeSwitchRadius(5).setTimeout(4));
        robot.turnAbsolute(180);
        flick(rings);
        robot.shooter.setVelocity(0);
    }
}