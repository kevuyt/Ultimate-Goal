package org.firstinspires.ftc.teamcode.Osiris.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.RingDetector;
import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqMath.MasqWayPoint;
import Library4997.MasqResources.MasqLinearOpMode;

import static Library4997.MasqRobot.OpMode.AUTO;
import static Library4997.MasqMath.MasqWayPoint.PointMode.*;
import static Library4997.MasqUtils.turnController;
import static org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.ZoneFinder.TargetZone.*;
import static org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.ZoneFinder.*;

/**
 * Created by Keval Kataria on 11/27/2020
 */
@Autonomous(name = "RedLegacy", group = "Osiris")
public class RedLegacy extends MasqLinearOpMode {
    private Osiris robot = new Osiris();
    private TargetZone zone;
    RingDetector detector = new RingDetector();
    private MasqWayPoint target = new MasqWayPoint().setTimeout(5).setSwitchMode(SWITCH).setTargetRadius(5).setAngularCorrectionSpeed(0.004).setPointSwitchRadius(24),
            strafe = new MasqWayPoint(-5,-30,0).setSwitchMode(TANK).setAngularCorrectionSpeed(0.002);

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap, AUTO);

        while(!opModeIsActive()) {
            zone = detector.findZone();

            dash.create("Remove download wire");
            dash.create("Zone: " + zone);
            dash.create("Control: " + detector.getControl());
            dash.create("Top: " + detector.getTop());
            dash.create("Bottom: " + detector.getBottom());
            dash.update();

            if(isStopRequested()) {
                robot.camera.stop();
                break;
            }
        }

        waitForStart();

        timeoutClock.reset();
        robot.camera.stop();
        robot.tracker.reset();

        robot.claw.lower();

        if (zone == A) target.setPoint(-7,-62,50);
        else if (zone == B) target.setPoint(-4,-85,0);
        else target.setPoint(-8,-110,42);

        robot.shooter.setVelocity(0.6);

        if(zone != A) robot.xyPath(strafe, target);
        else robot.xyPath(target);
        robot.turnAbsolute(target.getH(),1);

        robot.shooter.setVelocity(0.56);
        robot.claw.open();
        robot.hopper.setPosition(1);
        sleep();
        robot.claw.raise();

        double heading;
        if(zone == A) heading = 182;
        else if(zone == B) heading = 186;
        else heading = 182;

        robot.xyPath(new MasqWayPoint(7,-64, heading).setTimeout(5).setDriveCorrectionSpeed(0.008).setAngularCorrectionSpeed(0.03));
        robot.turnAbsolute(heading,5);
        flick();

        if(zone == A) heading = 180;
        else if(zone == B) heading = 184;
        else heading = 185;

        robot.xyPath(new MasqWayPoint(14, -64, heading).setTimeout(3).setDriveCorrectionSpeed(0.01).setAngularCorrectionSpeed(0.06));
        robot.turnAbsolute(heading);
        flick();

        if(zone == A) heading = 180;
        else if(zone == B) heading = 184;
        else heading = 180;

        robot.xyPath(new MasqWayPoint(23, -65, heading).setTimeout(3).setDriveCorrectionSpeed(0.01).setAngularCorrectionSpeed(0.06));
        robot.turnAbsolute(heading);
        flick();

        robot.shooter.setVelocity(0);
        robot.claw.lower();

        double wobbleY;
        if(zone == A) wobbleY = -28;
        else if(zone == B) wobbleY = -30;
        else wobbleY = -26;

        double wobbleX;
        if(zone == A) wobbleX = 24;
        else if(zone == B) wobbleX = 28.5;
        else wobbleX = 25;

        robot.xyPath(new MasqWayPoint(wobbleX, wobbleY, 180).setMinVelocity(0.27).setDriveCorrectionSpeed(0.02).setAngularCorrectionSpeed(0.05).setTimeout(5));

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

    private void flick() {
        robot.flicker.setPosition(0.9);
        sleep(1000);
        robot.flicker.setPosition(0);
        sleep();
    }
}