package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Thanos;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqPath;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqPoint;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/13/18.
 * Project: MasqLib
 */
@Autonomous(name = "PathFollowing", group = "test")
public class PathFollowing extends MasqLinearOpMode {
    Thanos robot = new Thanos();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("Waiting");
            dash.update();
        }
        waitForStart();
        List<MasqPoint> wayPoints = new ArrayList<>();
        wayPoints.add(robot.tracker.getPosition());
        wayPoints.add(new MasqPoint(wayPoints.get(0).getX() + 40, wayPoints.get(0).getY() + 40));
        robot.executePath(new MasqPath(wayPoints, 5), Direction.FORWARD, 0.6);
    }
}
