package org.firstinspires.ftc.teamcode.Osiris.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqMath.MasqVector;
import Library4997.MasqMath.MasqWayPoint;
import Library4997.MasqResources.MasqLinearOpMode;

import static Library4997.MasqRobot.OpMode.AUTO;
import static Library4997.MasqMath.MasqWayPoint.PointMode.*;
import static java.lang.Math.atan2;
import static java.lang.Math.toDegrees;
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
    public void runLinearOpMode() {
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

        if (zone == A) target.setPoint(-7,-63.5,70);
        else if (zone == B) target.setPoint(-3,-85,-20).setPointSwitchRadius(24);
        else target.setPoint(-8,-110,50);

        robot.shooter.setVelocity(0.6);

        if(zone != A) robot.xyPath(strafe, target);
        else robot.xyPath(target);
        robot.turnAbsolute(target.getH(),1);

        robot.shooter.setVelocity(0.6);
        robot.claw.open();
        robot.hopper.setPosition(1);
        sleep();
        robot.claw.raise();

        robot.xyPath(new MasqWayPoint(7,-66,180).setTimeout(5).setDriveCorrectionSpeed(0.01).setAngularCorrectionSpeed(0.06));
        flick(1);

        robot.xyPath(new MasqWayPoint(14, -66, 180).setDriveCorrectionSpeed(0.01).setAngularCorrectionSpeed(0.06));
        flick(1);

        robot.xyPath(new MasqWayPoint(22, -66, 180).setDriveCorrectionSpeed(0.01).setAngularCorrectionSpeed(0.06));
        flick(1);

        robot.shooter.setVelocity(0);

        /*if(zone != A) {

            if(zone == C) {
                shootStack(3);
                robot.turnAbsolute(0);
            }
            shootStack(1);
        }
         */
        robot.claw.lower();
        robot.xyPath(new MasqWayPoint(robot.tracker.getGlobalX(), -75, 0).setSwitchMode(SWITCH).setNoHeading(true).setDriveCorrectionSpeed(0.025));
    }

    private void flick(int iterations) {
        for (int i = 0; i < iterations; i++) {
            robot.flicker.setPosition(0.9);
            sleep(1000);

            robot.flicker.setPosition(0);
            sleep();
        }
    }
    private void shootStack(int rings) {
        MasqWayPoint stack = new MasqWayPoint(-6, -44, robot.tracker.getHeading()).setTimeout(3).setSwitchMode(SWITCH).setAngularCorrectionSpeed(0.12);
        MasqVector displacement = robot.getCurrentWayPoint().getPoint().displacement(stack.getPoint());
        robot.turnAbsolute(90 - toDegrees(atan2(displacement.getY(), displacement.getX())));
        robot.hopper.setPosition(0);
        robot.intake.setVelocity(-1);
        robot.xyPath(stack);
        robot.shooter.setVelocity(0.82);
        sleep(500);
        robot.intake.setVelocity(0);
        robot.turnAbsolute(180);
        robot.hopper.setPosition(1);
        robot.xyPath(new MasqWayPoint(9, -60, 180).setSwitchMode(SWITCH).setModeSwitchRadius(5));
        flick(rings);
        robot.shooter.setVelocity(0);
    }
}