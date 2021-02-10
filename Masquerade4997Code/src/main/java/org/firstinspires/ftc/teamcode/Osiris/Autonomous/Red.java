package org.firstinspires.ftc.teamcode.Osiris.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqSensors.MasqPositionTracker.MasqWayPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqSensors.MasqPositionTracker.MasqWayPoint.PointMode.MECH;
import static org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.ZoneFinder.TargetZone.*;
import static org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.ZoneFinder.*;

/**
 * Created by Keval Kataria on 11/27/2020
 */
@Autonomous(name = "Red", group = "Osiris")
public class Red extends MasqLinearOpMode {
    private Osiris robot = new Osiris();
    private TargetZone zone;
    MasqWayPoint target = new MasqWayPoint().setTimeout(5).setTargetRadius(4).setOnComplete(() -> robot.claw.open()), strafe = new MasqWayPoint(-10,-36,0).setSwitchMode(MECH).setTimeout(5);

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

        robot.camera.stop();


        robot.claw.lower();


        if (zone == A) target = target.setPoint(-10,-63.5,90);
        else if (zone == B) target = target.setPoint(4,-80,0);
        else target = target.setPoint(-10,-108,90);


        if(zone != A) robot.xyPath(7, strafe, target);
        else robot.xyPath(3,target);
        robot.stop();

        robot.claw.raise();
/*
        robot.xyPath(new MasqWayPoint(19,-60,170).setTimeout(3));
        shoot(1);
        robot.xyPath(new MasqWayPoint(26.5,-60,170));
        shoot(1);
        robot.xyPath(new MasqWayPoint(34,-60,170));
        shoot(1);
        robot.shooter.setVelocity(0);

        if(zone == B) shootStack(1);

        if(zone == C) {
            shootStack(3);
            shootStack(1);
        }

        robot.xyPath(1, new MasqWayPoint(robot.tracker.getGlobalX(), -64, 180));
         */
    }
    private void shoot(int iterations) {
        robot.shooter.setVelocity(1);

        for (int i = 0; i < iterations; i++) {
            robot.flicker.setPosition(1);
            sleep(0.5);
            robot.flicker.setPosition(0);
            sleep(0.5);
        }

        robot.shooter.setVelocity(0);
    }
    private void shootStack(int rings) {
        robot.intake.setVelocity(1);
        robot.xyPath(4, new MasqWayPoint(-2, -44, 180).setTimeout(4));
        robot.intake.setVelocity(0);

        robot.xyPath(2, new MasqWayPoint(9, -60, -10));
        shoot(rings);
    }
}