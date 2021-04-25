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

@Autonomous(preselectTeleOp = "RobotTeleOp", group = "Main")
public class Red extends MasqLinearOpMode {
    private final Osiris robot = new Osiris();
    TargetZone zone = C;
    private final MasqWayPoint target = new MasqWayPoint().setTimeout(5).setSwitchMode(SWITCH)
            .setTargetRadius(5).setAngularCorrectionSpeed(0.01).setModeSwitchRadius(24)
            .setName("Drop Zone").setDriveCorrectionSpeed(0.13);

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

        if(zone == A) A();
        else if(zone == B) B();
        else C();
    }

    public void A() {
        target.setPoint(13,58,45);
        firstWobble(false);
        powerShots();
        secondWobble(2,0,-20, false, false);
        park(true);
    }
    public void B() {
        target.setPoint(3,80,0).setMinVelocity(0.25);
        firstWobble(true);
        powerShots();
        secondWobble(0,5,-16,false, true);
        shootInGoal();
        park(false);
    }
    public void C() {
        target.setPoint(17,110,45);
        firstWobble(true);
        powerShots();
        secondWobble(5,10,-16,true, false);
        park(false);
    }

    public void firstWobble(boolean avoidStack) {
        MasqWayPoint strafe = new MasqWayPoint(7,30,0).setSwitchMode(TANK).setMinVelocity(0.8);

        robot.hopper.setPosition(1);
        robot.claw.mid();
        if (avoidStack) robot.xyPath(strafe, target);
        else robot.xyPath(target);

        robot.shooter.setPower(0.56);
        sleep();

        robot.claw.lower();
        robot.claw.open();
        robot.flicker.setPosition(0);
        sleep();
    }
    public void powerShots() {
        robot.compressor.setPosition(1);
        robot.claw.raise();

        robot.xyPath(new MasqWayPoint(-10,66, 0).setMinVelocity(0.2).setDriveCorrectionSpeed(0.06)
                .setTimeout(5).setAngularCorrectionSpeed(0.03).setName("First Power Shot"));
        robot.claw.close();
        sleep();
        flick();

        robot.xyPath(new MasqWayPoint(-20,66, 0).setMinVelocity(0.2).setDriveCorrectionSpeed(0.04)
                .setAngularCorrectionSpeed(0.03).setTimeout(4).setName("Second Power Shot"));
        sleep();
        flick();

        robot.xyPath(new MasqWayPoint(-27,66.5, 0).setMinVelocity(0.2).setDriveCorrectionSpeed(0.04)
                .setAngularCorrectionSpeed(0.03).setTimeout(4).setName("Third Power Shot"));
        sleep();
        flick();

        robot.shooter.setPower(0);
        robot.hopper.setPosition(0);
        robot.compressor.setPosition(0);
    }
    public void secondWobble(double yDecrease, double xDecrease, double wobbleX, boolean avoidStack, boolean intakeStack) {
        robot.claw.mid();
        robot.claw.open();

        robot.turnAbsolute(170,1);
        robot.xyPath(new MasqWayPoint(wobbleX, 27, 180).setMinVelocity(0).setTimeout(5)
                .setName("Second Wobble Goal").setDriveCorrectionSpeed(0.03).setAngularCorrectionSpeed(0.02));

        robot.claw.lower();
        sleep();

        robot.claw.close();

        target.setSwitchMode(MECH).setAngularCorrectionSpeed(0.03)
                .setY(target.getY() - yDecrease).setX(target.getX() - xDecrease);
        MasqWayPoint back = new MasqWayPoint(robot.tracker.getGlobalX(), 50, robot.tracker.getHeading())
                .setSwitchMode(TANK).setAngularCorrectionSpeed(0.01);

        sleep(1000);
        robot.claw.mid();

        if(intakeStack) {
            MasqWayPoint starterStack = new MasqWayPoint(-5, 40, 0).setSwitchMode(TANK)
                    .setDriveCorrectionSpeed(0.04).setAngularCorrectionSpeed(0.02).setName("Starter Stack");

            robot.intake.setPower(1);
            robot.turnAbsolute(30);
            robot.xyPath(starterStack, target);
        }
        else if(avoidStack) robot.xyPath(back, target);
        else robot.xyPath(target);
        sleep();

        robot.claw.open();
        sleep();
        robot.claw.raise();
    }
    public void shootInGoal() {
        robot.intake.setPower(0);
        robot.shooter.setPower(0.7);
        robot.hopper.setPosition(1);
        robot.xyPath(new MasqWayPoint(0,65, 0).setMinVelocity(0.2).setDriveCorrectionSpeed(0.04)
                .setTimeout(5).setAngularCorrectionSpeed(0.04));
        robot.compressor.setPosition(1);
        sleep(1000);
        flick();
        robot.shooter.setPower(0);
    }
    public void park(boolean backUp) {
        MasqWayPoint park = new MasqWayPoint(robot.tracker.getGlobalX(), 75, robot.tracker.getHeading())
                .setDriveCorrectionSpeed(0.025).setAngularCorrectionSpeed(0.03).setName("Park");
        MasqWayPoint exit = new MasqWayPoint(robot.tracker.getGlobalX() - 25, 50, robot.tracker.getHeading());

        if(backUp) robot.xyPath(exit, park.setX(robot.tracker.getGlobalX() - 25));
        else robot.xyPath(park);

        robot.claw.lower();
        dash.create("Time Left:", 30 - timeoutClock.milliseconds() / 1000.0);
        dash.update();
        sleep((long) (30e3 - timeoutClock.milliseconds()));
    }

    private void flick() {
        robot.flicker.setPosition(1);
        sleep();
        robot.flicker.setPosition(0);
        sleep();
    }
}