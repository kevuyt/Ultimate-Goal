package org.firstinspires.ftc.teamcode.Osiris.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqSensors.MasqPositionTracker.MasqWayPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

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
            strafe = new MasqWayPoint(-7,-36,0).setSwitchMode(TANK).setAngularCorrectionSpeed(0.0015);

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initCamera(hardwareMap);

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

        if(zone != A) robot.xyPath(strafe, target);
        else robot.xyPath(target);
        robot.turnAbsolute(-target.getH(),1);

        robot.claw.open();
        robot.shooter.setVelocity(0.52);
        robot.hopper.setPosition(1);
        sleep(1.0);
        robot.claw.raise();

        robot.xyPath(new MasqWayPoint(19,-53,190).setTimeout(4).setDriveCorrectionSpeed(0.1));
        robot.turnAbsolute(170);
        flick(1);

        robot.xyPath(new MasqWayPoint(25.7, -52.3, 190));
        robot.turnAbsolute(170);
        flick(1);

        robot.xyPath(new MasqWayPoint(39, -51.6, 190));
        robot.turnAbsolute(170);
        flick(1);

        robot.shooter.setVelocity(0);

        /*if(zone == B) shootStack(1);

        if(zone == C) {
            shootStack(3);
            shootStack(1);
        }

        robot.xyPath(1, new MasqWayPoint(robot.tracker.getGlobalX(), -72, -robot.tracker.getHeading()).setSwitchMode(TANK));
        /**/

        while(timeoutClock.hasNotPassed(29.5)) {
            dash.create(robot.tracker);
            dash.update();
        }

    }

    private void flick(int iterations) {
        for (int i = 0; i < iterations; i++) {
            robot.flicker.setPosition(1);
            sleep(1.0);

            robot.flicker.setPosition(0);
            sleep(1.0);
        }
    }
    private void shootStack(int rings) {
        robot.hopper.setPosition(0);
        robot.intake.setVelocity(-1);
        robot.xyPath(4, new MasqWayPoint(-6, -44,0).setTimeout(4).setSwitchMode(SWITCH).setAngularCorrectionSpeed(0.12));
        robot.stop();
        robot.intake.setVelocity(0);

        robot.hopper.setPosition(1);
        robot.shooter.setVelocity(0.7);
        robot.xyPath(2, new MasqWayPoint(9, -60, 190).setSwitchMode(SWITCH).setModeSwitchRadius(5).setTimeout(4));
        robot.stop();
        robot.turnAbsolute(190);
        flick(rings);
        robot.shooter.setVelocity(0);
    }
}