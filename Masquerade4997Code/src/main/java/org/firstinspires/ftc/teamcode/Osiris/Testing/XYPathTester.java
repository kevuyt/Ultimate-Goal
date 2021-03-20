package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import MasqueradeLibrary.MasqOdometry.MasqWayPoint;
import MasqueradeLibrary.MasqResources.MasqLinearOpMode;

import static MasqueradeLibrary.MasqRobot.OpMode.TELEOP;

/**
 * Created by Keval Kataria on 12/30/2020
 */

@TeleOp(name = "XYPathTester", group = "Test")
@Disabled
public class XYPathTester extends MasqLinearOpMode {
    private Osiris robot = new Osiris();
    private double heading = 0;

    @Override
    public void runLinearOpMode() {
        robot.init(TELEOP);

        dash.create(robot.tracker);
        dash.update();

        waitForStart();


        while(opModeIsActive()) {
            if(gamepad1.a) robot.xyPath(new MasqWayPoint(0,0, 0));
            if(gamepad1.dpad_up) robot.xyPath(new MasqWayPoint(0,24, heading));
            if(gamepad1.dpad_down) robot.xyPath(new MasqWayPoint(0,-24, heading));
            if(gamepad1.dpad_left) robot.xyPath(new MasqWayPoint(-24, 0, heading));
            if(gamepad1.dpad_right) robot.xyPath(new MasqWayPoint(24,0, heading));
            if(gamepad1.b) heading += 0.1;

            dash.create(robot.tracker);
            dash.create("Press A to reset to 0,0");
            dash.create("Use the d pad to choose which tile to move to");
            dash.create("Press B to increase the heading. Current Heading: ", heading);
            dash.create("Note the heading will not update until it moves to one of the 4 positions.");
            dash.create("Resetting will not reset the heading but will go temporarily back to the starting heading.");
            dash.update();
        }
    }
}