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
    MasqWayPoint target = new MasqWayPoint(), strafe = new MasqWayPoint().setPoint(-10,-24,0).setSwitchMode(MECH);


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

        robot.claw.lower();

        if (zone == A) target = target.setPoint(-10,-60,90);
        else if (zone == B) target = target.setPoint(14,-84,90);
        else target = target.setPoint(-10,-108,90);

        robot.xyPath(4, strafe, target.setOnComplete(() -> robot.claw.open()));
        robot.stop();
    }
}