package org.firstinspires.ftc.teamcode.Osiris.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Osiris.Autonomous.RingDetector.TargetZone;
import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import MasqLibrary.MasqOdometry.MasqWayPoint;
import MasqLibrary.MasqResources.MasqLinearOpMode;

import static MasqLibrary.MasqOdometry.MasqWayPoint.PointMode.*;
import static MasqLibrary.MasqResources.MasqUtils.turnController;
import static MasqLibrary.MasqRobot.OpMode.AUTO;
import static org.firstinspires.ftc.teamcode.Osiris.Autonomous.RingDetector.TargetZone.*;

/**
 * Created by Keval Kataria on 3/8/2021
 */

@Autonomous
public class Red extends MasqLinearOpMode {
    private Osiris robot = new Osiris();
    private TargetZone zone;
    private MasqWayPoint target = new MasqWayPoint().setTimeout(5).setSwitchMode(SWITCH).setTargetRadius(5)
            .setAngularCorrectionSpeed(0.004).setPointSwitchRadius(24).setName("Drop Zone").setDriveCorrectionSpeed(0.16),
            strafe = new MasqWayPoint(5,30,0).setSwitchMode(TANK).setMinVelocity(0.8);

    @Override
    public void runLinearOpMode() {
        robot.init(AUTO);
        RingDetector detector = (RingDetector) robot.camera.detector;

        while (!opModeIsActive()) {
            zone = detector.findZone();

            dash.create("Zone:", zone);
            dash.create("Control:",  detector.getControl());
            dash.create("Top:",  detector.getTop());
            dash.create("Bottom:",  detector.getBottom());
            dash.update();

            if (isStopRequested()) {
                robot.camera.stop();
                break;
            }
        }

        waitForStart();

        timeoutClock.reset();
        robot.camera.stop();
        robot.tracker.reset();

        if(zone == A) {
            target.setPoint(7,62,50);
        }
        else if(zone == B) {
            target.setPoint(0,82,30);
        }
        else target.setPoint(8,110,42);

        robot.claw.mid();

        //robot.shooter.setPower(0.6);

        if(zone != A) robot.xyPath(strafe, target);
        else robot.xyPath(target);
        sleep();

        robot.claw.open();
        robot.claw.lower();

        robot.xyPath(new MasqWayPoint(-13, 60, 90).setMinVelocity(0.8),
                new MasqWayPoint(-20, 28, 175).setMinVelocity(0.27).setDriveCorrectionSpeed(0.05)
                        .setTimeout(5).setAngularCorrectionSpeed(0.22).setName("Second Wobble Goal"));

        robot.claw.close();
        sleep(700);

        target.setSwitchMode(MECH).setAngularCorrectionSpeed(0.20);
        MasqWayPoint back = new MasqWayPoint(robot.tracker.getGlobalX(), 50, robot.tracker.getHeading()).setSwitchMode(TANK);

        robot.claw.mid();

        if(zone == A) robot.xyPath(target);
        else robot.xyPath(back, target);

        robot.claw.open();
        sleep();
        robot.claw.raise();

        MasqWayPoint park = new MasqWayPoint(robot.tracker.getGlobalX(), 72, robot.tracker.getHeading())
                .setDriveCorrectionSpeed(0.025).setAngularCorrectionSpeed(0.03).setName("Park");
        if(zone != B) park.setX(park.getX() - 10);
        robot.xyPath(park);

        robot.claw.raise();

        /*robot.xyPath(new MasqWayPoint(-7,64, 180).setTimeout(5).setDriveCorrectionSpeed(0.008)
                .setAngularCorrectionSpeed(0.07));
        shoot(1);

        robot.xyPath(new MasqWayPoint(-7,64, 180).setTimeout(5).setDriveCorrectionSpeed(0.008)
                .setAngularCorrectionSpeed(0.07));
        shoot(1);

        robot.xyPath(new MasqWayPoint(-7,64, 180).setTimeout(5).setDriveCorrectionSpeed(0.008)
                .setAngularCorrectionSpeed(0.07));
        shoot(1);

        robot.shooter.setPower(0);
        robot.claw.lower();

        robot.xyPath(new MasqWayPoint(-24, 28, 180).setMinVelocity(0.27).setDriveCorrectionSpeed(0.02)
                .setAngularCorrectionSpeed(0.05).setTimeout(5).setName("Second Wobble Goal"));

        robot.claw.close();
        sleep();

        target.setH(target.getH() + (zone == B ? -15 : 25)).setSwitchMode(MECH);
        MasqWayPoint back = new MasqWayPoint(robot.tracker.getGlobalX(), 50, robot.tracker.getHeading()).setSwitchMode(TANK);

        robot.claw.mid();

        if(zone == A) robot.xyPath(target);
        else robot.xyPath(back, target);
        turnController.setKp(0.02);
        robot.turnAbsolute(target.getH(),3);
        robot.claw.open();
        sleep();
        robot.claw.raise();

        MasqWayPoint park = new MasqWayPoint(robot.tracker.getGlobalX(), 72, robot.tracker.getHeading())
                .setDriveCorrectionSpeed(0.025).setAngularCorrectionSpeed(0.03).setName("Park");
        if(zone != B) park.setX(park.getX() - 10);
        robot.xyPath(park);

         */
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