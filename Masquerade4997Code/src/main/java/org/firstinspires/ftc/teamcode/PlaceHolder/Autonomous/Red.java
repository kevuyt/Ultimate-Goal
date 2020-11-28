package org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PlaceHolder.Robot.PlaceHolder;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint.PointMode.MECH;
import static org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision.ZoneFinder.TargetZone.*;
import static org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision.ZoneFinder.*;

/**
 * Created by Keval Kataria on 11/27/2020
 */
@Autonomous(name = "Red", group = "PlaceHolder")
public class Red extends MasqLinearOpMode {
    private PlaceHolder robot = new PlaceHolder();
    private TargetZone zone;

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initCamera(hardwareMap);

        while(!opModeIsActive()) {
            zone = findZone(robot.camera.detector);
            dash.create("Zone: " + zone);
            dash.create("Control" + robot.detector.getControl());
            dash.create("Average" + robot.detector.getAverage());
            dash.update();
        }

        waitForStart();

        robot.camera.stop();

        MasqWayPoint target, strafe = new MasqWayPoint().setPoint(-24,0,0).setSwitchMode(MECH);

        if (zone == A) target = new MasqWayPoint().setPoint(-24,-60,90);
        else if (zone == B) target = new MasqWayPoint().setPoint(-7,-84,90);
        else target = new MasqWayPoint().setPoint(-24,-108,90);

        robot.xyPath(4, strafe, target);
    }
}