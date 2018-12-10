package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robots.TestBot;

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
@Disabled
public class PathFollowing extends MasqLinearOpMode {
    TestBot robot = new TestBot();
    @Override
    public void runLinearOpMode() {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.tracker.getPosition());
            dash.update();
        }
        waitForStart();
        List<MasqPoint> wayPoints = new ArrayList<>();
        wayPoints.add(robot.tracker.getPosition());
        wayPoints.add(new MasqPoint(wayPoints.get(0).getX(), wayPoints.get(0).getY() + 10));
        robot.executePath(new MasqPath(wayPoints, 5), Direction.FORWARD, 0.6);
    }
}
