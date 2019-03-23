package org.firstinspires.ftc.teamcode.Robots.Kenya.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Kenya.Kenya;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 3/15/19.
 * Project: MasqLib
 */
@TeleOp(name = "NFS", group = "Test")
public class NFS extends MasqLinearOpMode {
    Kenya robot;
    @Override
    public void runLinearOpMode() {
        robot = new Kenya();
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }
        waitForStart();
        while(opModeIsActive()) {
            robot.NFS(controller1);
            if (controller1.a()) {
                robot.wing1.setPosition(0.9);
                robot.wing2.setPosition(0.9);
            }
            else {
                robot.wing1.setPosition(0.1);
                robot.wing2.setPosition(0.1);
            }
        }
    }
}
