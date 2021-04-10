package org.firstinspires.ftc.teamcode.Osiris.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Osiris.Autonomous.RingDetector.TargetZone;
import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import MasqLibrary.MasqOdometry.MasqWayPoint;
import MasqLibrary.MasqResources.MasqLinearOpMode;

import static MasqLibrary.MasqOdometry.MasqWayPoint.PointMode.*;
import static MasqLibrary.MasqRobot.OpMode.AUTO;
import static org.firstinspires.ftc.teamcode.Osiris.Autonomous.RingDetector.TargetZone.*;

/**
 * Created by Keval Kataria on 3/8/2021
 */

@Autonomous(/*preselectTeleOp = "RobotTeleOp", */group = "Main")
public class Red extends MasqLinearOpMode {
    private final Osiris robot = new Osiris();
    private TargetZone zone;
    private final MasqWayPoint target = new MasqWayPoint().setTimeout(5).setSwitchMode(SWITCH)
            .setTargetRadius(5).setAngularCorrectionSpeed(0.01).setModeSwitchRadius(24)
            .setName("Drop Zone").setDriveCorrectionSpeed(0.16),
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

        robot.shooter.setPower(0.75);
        robot.compressor.setPosition(1);
        robot.hopper.setPosition(1);

        if(zone == A) target.setPoint(17,64,45);
        else if(zone == B) target.setPoint(-5,75,0).setMinVelocity(0.25);
        else target.setPoint(17,115,45);

        if(zone == B) {
            robot.claw.lower();
            robot.xyPath(new MasqWayPoint(0, 15,5).setMinVelocity(0).setTimeout(4).setDriveCorrectionSpeed(0.04).setAngularCorrectionSpeed(0.04));
            sleep(1000);
            flick();
            robot.claw.raise();
            robot.hopper.setPosition(0);
            robot.compressor.setPosition(0);
            robot.shooter.setPower(0);
            robot.intake.setPower(1);
            robot.xyPath(new MasqWayPoint(-10, 30, 0).setOnComplete(() -> robot.claw.mid()).setDriveCorrectionSpeed(0.06), target);
        }
        else {
            robot.claw.mid();
            if (zone == C) robot.xyPath(strafe, target);
            else robot.xyPath(target);
        }
        robot.claw.lower();
        robot.claw.open();
        sleep();

        robot.hopper.setPosition(1);
        robot.compressor.setPosition(1);
        robot.shooter.setPower(0.7);

        robot.xyPath(new MasqWayPoint(-12,66, 0).setMinVelocity(0).setDriveCorrectionSpeed(0.08).setTimeout(4).setAngularCorrectionSpeed(0.07));
        flick();

        robot.xyPath(new MasqWayPoint(-20,66, 0).setMinVelocity(0).setDriveCorrectionSpeed(0.04)
                .setAngularCorrectionSpeed(0.03));
        flick();

        robot.xyPath(new MasqWayPoint(-28,66.5, 0).setMinVelocity(0).setDriveCorrectionSpeed(0.04)
                .setAngularCorrectionSpeed(0.03));
        flick();

        robot.shooter.setPower(0);
        robot.claw.lower();
        robot.claw.open();

        /*
        robot.turnAbsolute(180,1);
        robot.xyPath(new MasqWayPoint(zone == A ? -20 : -21, 29, 180).setMinVelocity(0).setTimeout(5)
                .setName("Second Wobble Goal").setDriveCorrectionSpeed(0.04));
        sleep();

        robot.claw.close();

        target.setSwitchMode(MECH).setAngularCorrectionSpeed(0.3).setY(target.getY() - (zone == A ? 2 : 5)).setX(target.getX() - (zone == A ? 0 : 10));
        MasqWayPoint back = new MasqWayPoint(robot.tracker.getGlobalX(), 50, robot.tracker.getHeading()).setSwitchMode(TANK).setAngularCorrectionSpeed(0.01);

        sleep(1000);
        robot.claw.mid();

        if(zone == A) robot.xyPath(target);
        else robot.xyPath(back, target);
        sleep();

        robot.claw.open();
        sleep();
        robot.claw.raise();

        MasqWayPoint park = new MasqWayPoint(robot.tracker.getGlobalX() - (zone == A ? 25 : 0), 72, robot.tracker.getHeading())
                .setDriveCorrectionSpeed(0.025).setAngularCorrectionSpeed(0.03).setName("Park");
        MasqWayPoint exit = new MasqWayPoint(robot.tracker.getGlobalX() - 25, 50, robot.tracker.getHeading());
        if(zone == A) robot.xyPath(exit, park);
        else robot.xyPath(park);

        robot.claw.init();
        sleep();

         */
    }

    private void flick() {
        robot.flicker.setPosition(1);
        sleep(750);
        robot.flicker.setPosition(0);
        sleep();
    }
}