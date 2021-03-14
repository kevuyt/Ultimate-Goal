package org.firstinspires.ftc.teamcode.Osiris.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.RingDetector;
import org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.RingDetector.TargetZone;
import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqMath.MasqWayPoint;
import Library4997.MasqResources.MasqLinearOpMode;

import static Library4997.MasqMath.MasqWayPoint.PointMode.*;
import static Library4997.MasqRobot.OpMode.AUTO;
import static Library4997.MasqUtils.turnController;
import static org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.RingDetector.TargetZone.*;

/**
 * Created by Keval Kataria on 3/8/2021
 */
@Autonomous(name = "Red", group = "Osiris")
public class Red extends MasqLinearOpMode {
    private Osiris robot = new Osiris();
    private TargetZone zone;
    private RingDetector detector;
    int iterations;
    private MasqWayPoint target = new MasqWayPoint().setTimeout(5).setSwitchMode(SWITCH).setTargetRadius(5).setAngularCorrectionSpeed(0.004).setPointSwitchRadius(24),
            strafe = new MasqWayPoint(-5,-30,0).setSwitchMode(TANK).setAngularCorrectionSpeed(0.002),
            stack = new MasqWayPoint(4, 30, 0).setSwitchMode(TANK);

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap, AUTO);
        detector = (RingDetector) robot.camera.detector;

        while (!opModeIsActive()) {
            zone = detector.findZone();

            dash.create("Zone: " + zone);
            dash.create("Control: " + detector.getControl());
            dash.create("Top: " + detector.getTop());
            dash.create("Bottom: " + detector.getBottom());
            dash.update();

            if (isStopRequested()) {
                robot.camera.stop();
                return;
            }
        }

        waitForStart();

        timeoutClock.reset();
        robot.camera.stop();
        robot.tracker.reset();

        robot.claw.raise();

        if(zone == B) {
            iterations = 1;
            target.setPoint(-4,-85,0);
        }
        else if(zone == C) {
            iterations = 3;
            target.setPoint(-8,-110,42);
        }
        else target.setPoint(-7,-62,50);

        robot.shooter.setVelocity(1);
        shoot(iterations);

        if(zone != A) {
            robot.intake.setVelocity(1);
            robot.xyPath(stack);
            if(zone == B) while(robot.getRings() < 1) sleep(100);
            else {
                robot.shooter.setVelocity(1);
                while(robot.getRings() < 2) sleep(100);
                shoot(1);
                robot.shooter.setVelocity(0);
                while(robot.getRings() < 3) sleep(100);
            }
        }

        robot.claw.mid();

        robot.shooter.setVelocity(0.6);

        if(zone != A) robot.xyPath(strafe, target);
        else robot.xyPath(target);
        robot.turnAbsolute(target.getH(),1);

        robot.xyPath(new MasqWayPoint(7,-64, 180).setTimeout(5).setDriveCorrectionSpeed(0.008).setAngularCorrectionSpeed(0.03));
        robot.turnAbsolute(180,5);
        shoot(1);

        robot.xyPath(new MasqWayPoint(7,-64, 180).setTimeout(5).setDriveCorrectionSpeed(0.008).setAngularCorrectionSpeed(0.03));
        robot.turnAbsolute(180,5);
        shoot(1);

        robot.xyPath(new MasqWayPoint(7,-64, 180).setTimeout(5).setDriveCorrectionSpeed(0.008).setAngularCorrectionSpeed(0.03));
        robot.turnAbsolute(180,5);
        shoot(1);

        robot.shooter.setVelocity(0);
        robot.claw.lower();

        robot.xyPath(new MasqWayPoint(24, -28, 180).setMinVelocity(0.27).setDriveCorrectionSpeed(0.02).setAngularCorrectionSpeed(0.05).setTimeout(5));

        robot.claw.close();
        sleep();

        target.setH(target.getH() + (zone == B ? -15 : 25)).setSwitchMode(MECH);
        MasqWayPoint back = new MasqWayPoint(robot.tracker.getGlobalX(), -50, robot.tracker.getHeading()).setSwitchMode(TANK);

        robot.claw.mid();

        if(zone == A) robot.xyPath(target);
        else robot.xyPath(back, target);
        turnController.setKp(0.02);
        robot.turnAbsolute(target.getH(),3);
        robot.claw.open();
        sleep();
        robot.claw.raise();

        MasqWayPoint park = new MasqWayPoint(robot.tracker.getGlobalX(), -72, robot.tracker.getHeading()).setDriveCorrectionSpeed(0.025).setAngularCorrectionSpeed(0.03);
        if(zone != B) park.setX(park.getX() + 10);
        robot.xyPath(park);
    }

    private void shoot(int iterations) {
        for (int i = 0; i < iterations; i++) {
            robot.flicker.setPosition(0.9);
            sleep();
            robot.flicker.setPosition(0);
            sleep();
        }
    }
}