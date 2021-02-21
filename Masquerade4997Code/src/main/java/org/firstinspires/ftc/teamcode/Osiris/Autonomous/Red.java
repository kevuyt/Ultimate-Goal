package org.firstinspires.ftc.teamcode.Osiris.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqSensors.MasqPositionTracker.MasqWayPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqRobot.OpMode.AUTO;
import static Library4997.MasqSensors.MasqPositionTracker.MasqWayPoint.PointMode.*;
import static org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.ZoneFinder.TargetZone.*;
import static org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.ZoneFinder.*;

/**
 * Created by Keval Kataria on 11/27/2020
 */
@Autonomous(name = "Red", group = "Osiris")
public class Red extends MasqLinearOpMode {
    private Osiris robot = new Osiris();
    private TargetZone zone;
    private MasqWayPoint target = new MasqWayPoint().setTimeout(5).setSwitchMode(SWITCH).setTargetRadius(5).setAngularCorrectionSpeed(0.002),
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

        robot.claw.lower();

        if (zone == A) target = target.setPoint(-7,-63.5,70);
        else if (zone == B) target = target.setPoint(-3,-80,-20);
        else target = target.setPoint(-10,-103,30);

        robot.shooter.setVelocity(0.52);

        if(zone != A) robot.xyPath(strafe, target);
        else robot.xyPath(target);
        robot.turnAbsolute(-target.getH(),1);

        robot.shooter.setVelocity(0.52);
        robot.claw.open();
        robot.hopper.setPosition(1);
        sleep(1.0);
        robot.claw.raise();

        robot.xyPath(new MasqWayPoint(12,-67,180).setTimeout(4).setDriveCorrectionSpeed(0.2));
        robot.turnAbsolute(180);
        flick(1);

        robot.xyPath(new MasqWayPoint(20, -67, 180));
        robot.turnAbsolute(180);
        flick(1);

        robot.xyPath(new MasqWayPoint(31, -67, 180));
        robot.turnAbsolute(180);
        flick(1);

        robot.shooter.setVelocity(0);

        if(zone == B) {
            robot.turnAbsolute(45);
            shootStack(1);
        }

        if(zone == C) {
            robot.turnAbsolute(45);
            shootStack(3);
            shootStack(1);
        }

        /**/
        robot.xyPath(new MasqWayPoint(robot.tracker.getGlobalX(), -72, -robot.tracker.getHeading()));

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
        robot.xyPath(new MasqWayPoint(-6, -44,0).setTimeout(4).setSwitchMode(SWITCH).setAngularCorrectionSpeed(0.12));
        robot.intake.setVelocity(0);
        robot.turnAbsolute(0);

        robot.hopper.setPosition(1);
        robot.shooter.setVelocity(0.6);
        robot.xyPath(new MasqWayPoint(9, -60, 190).setSwitchMode(SWITCH).setModeSwitchRadius(5).setTimeout(4));
        robot.stop();
        robot.turnAbsolute(190);
        flick(rings);
        robot.shooter.setVelocity(0);
    }
}