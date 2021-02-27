package org.firstinspires.ftc.teamcode.Osiris.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
@Autonomous(name = "Red", group = "Osiris")
public class Red extends MasqLinearOpMode {
    private Osiris robot = new Osiris();
    private TargetZone zone;
    private MasqWayPoint target = new MasqWayPoint().setTimeout(5).setSwitchMode(SWITCH).setTargetRadius(5).setAngularCorrectionSpeed(0.004),
            strafe = new MasqWayPoint(-5,-30,0).setSwitchMode(TANK).setAngularCorrectionSpeed(0.002);

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap, AUTO);

        while(!opModeIsActive()) {
            zone = findZone(robot.camera.detector);

            dash.create("Remove download wire");
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

        if (zone == A) target.setPoint(-7,-62,65);
        else if (zone == B) target.setPoint(-3,-87,-10).setPointSwitchRadius(24);
        else target.setPoint(-8,-110,45);

        robot.shooter.setVelocity(0.6);

        if(zone != A) robot.xyPath(strafe, target);
        else robot.xyPath(target);
        robot.turnAbsolute(target.getH(),1);

        robot.shooter.setVelocity(0.6);
        robot.claw.open();
        robot.hopper.setPosition(1);
        sleep();
        robot.claw.raise();

        int heading = zone == B ? 185 : 182;

        robot.xyPath(new MasqWayPoint(7,-64, heading).setTimeout(5).setDriveCorrectionSpeed(0.007).setAngularCorrectionSpeed(0.03));
        robot.turnAbsolute(heading,5);
        flick();

        heading = zone == B ? 183 : 180;

        robot.xyPath(new MasqWayPoint(14, -64, heading).setTimeout(3).setDriveCorrectionSpeed(0.01).setAngularCorrectionSpeed(0.06));
        robot.turnAbsolute(heading);
        flick();

        heading = zone == B ? 184 : 181;

        robot.xyPath(new MasqWayPoint(23, -64, heading).setTimeout(3).setDriveCorrectionSpeed(0.01).setAngularCorrectionSpeed(0.06));
        robot.turnAbsolute(heading);
        flick();

        robot.shooter.setVelocity(0);
        robot.claw.mid();

        robot.xyPath(new MasqWayPoint(zone == B ? 28 : 23, -29, 180).setDriveCorrectionSpeed(0.002).setAngularCorrectionSpeed(0.05).setTimeout(3));

        robot.claw.close();
        sleep();

        target.setAngularCorrectionSpeed(zone == A ? 0.08 : 0.12).setY(target.getY() + 4).setH(target.getH() + (zone == B ? -15 : 30));
        MasqWayPoint back = new MasqWayPoint(robot.tracker.getGlobalX(), -50, robot.tracker.getHeading()).setSwitchMode(TANK);

        if(zone == A) robot.xyPath(target);
        else robot.xyPath(back, target);
        turnController.setKp(0.03);
        robot.turnAbsolute(target.getH(),3);
        robot.claw.open();
        sleep();

        MasqWayPoint park = new MasqWayPoint(robot.tracker.getGlobalX(), -75, robot.tracker.getHeading()).setDriveCorrectionSpeed(0.025).setAngularCorrectionSpeed(0.03);
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