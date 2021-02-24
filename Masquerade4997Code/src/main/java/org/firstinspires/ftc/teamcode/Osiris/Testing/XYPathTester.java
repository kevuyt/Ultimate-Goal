package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqMath.MasqWayPoint;
import Library4997.MasqResources.MasqLinearOpMode;

import static Library4997.MasqRobot.OpMode.TELEOP;

/**
 * Created by Keval Kataria on 12/30/2020
 */

@TeleOp(name = "XYPathTester", group = "Test")
public class XYPathTester extends MasqLinearOpMode {
    private Osiris robot = new Osiris();

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap, TELEOP);

        dash.create(robot.tracker);
        dash.update();

        waitForStart();

        robot.xyPath(new MasqWayPoint(24,0,0));

        while(opModeIsActive()) {
            robot.tracker.updateSystem();
            dash.create(robot.tracker);
            dash.update();
        }
    }
}