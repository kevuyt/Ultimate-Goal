package org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous;

import org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision.ZoneFinder;
import org.firstinspires.ftc.teamcode.PlaceHolder.Robot.PlaceHolder;


import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision.ZoneFinder.TargetZone.*;
import static org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision.ZoneFinder.*;

/**
 * Created by Keval Kataria on 11/27/2020
 */
public class Red extends MasqLinearOpMode {
    private PlaceHolder robot = new PlaceHolder();
    private TargetZone zone;
    private MasqWayPoint target;

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initCamera(hardwareMap);

        while(!opModeIsActive()) {
            zone = findZone(robot.camera.detector);
            dash.create("Rings: " + zone);
            dash.update();
        }

        waitForStart();

        if (zone == A) target = new MasqWayPoint().setPoint(-24,-60,90);
        else if (zone == B) target = new MasqWayPoint().setPoint(-7,-84,90);
        else target = new MasqWayPoint().setPoint(-24,-108,90);
        robot.xyPath(4,target);
    }
}