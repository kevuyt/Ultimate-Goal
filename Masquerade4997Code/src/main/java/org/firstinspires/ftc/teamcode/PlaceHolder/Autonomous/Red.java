package org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PlaceHolder.Robot.PlaceHolder;

import Library4997.MasqSensors.MasqPositionTracker.MasqWayPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqSensors.MasqPositionTracker.MasqWayPoint.PointMode.MECH;
import static org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision.ZoneFinder.TargetZone.*;
import static org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision.ZoneFinder.*;

/**
 * Created by Keval Kataria on 11/27/2020
 */
@Autonomous(name = "Red", group = "PlaceHolder")
public class Red extends MasqLinearOpMode {
    private PlaceHolder robot = new PlaceHolder();
    private TargetZone zone;
    MasqWayPoint target = new MasqWayPoint().setTimeout(5), strafe = new MasqWayPoint().setPoint(-14,-36,0).setSwitchMode(MECH);

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initCamera(hardwareMap);

        while(!opModeIsActive()) {
            zone = findZone(robot.camera.detector);
            dash.create("Zone: " + zone);
            dash.create("Control: " + robot.detector.getControl());
            dash.create("Top: " + robot.detector.getTop());
            dash.create("Bottom: ", robot.detector.getBottom());
            dash.update();
        }

        waitForStart();

        robot.camera.stop();

        robot.claw.lower();

        if (zone == A) target = target.setPoint(-10,-60,90);
        else if (zone == B) target = target.setPoint(-4,-84,0);
        else target = target.setPoint(-10,-108,90);

        if(zone != A) robot.xyPath(7, strafe, target.setOnComplete(() -> robot.claw.open()));
        else robot.xyPath(3,target.setOnComplete(() -> robot.claw.open()));
        robot.stop();

        robot.xyPath(3,new MasqWayPoint().setPoint(19,-60,0).setTimeout(3).setOnComplete(() -> robot.shoot(1)));
        robot.xyPath(2,new MasqWayPoint().setPoint(26.5,-60,0).setOnComplete(() -> robot.shoot(1)));
        robot.xyPath(2,new MasqWayPoint().setPoint(34,-60,0).setOnComplete(() -> robot.shoot(1)));
    }
}