package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PlaceHolder.Robot.PlaceHolder;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint.PointMode.MECH;

/**
 * Created by Keval Kataria on 11/11/2020
 */
@Autonomous(name = "TestAuto", group = "PlaceHolder")
public class TestAuto extends MasqLinearOpMode {
    private PlaceHolder robot = new PlaceHolder();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();

        while(!opModeIsActive()) {
            dash.create("Something for vision");
            dash.update();
        }

        waitForStart();

        robot.xyPathV2(2, new MasqWayPoint().setPoint(0,22.25, 90).setSwitchMode(MECH), new MasqWayPoint().setPoint(22.25, 22.25,-90));
    }
}
