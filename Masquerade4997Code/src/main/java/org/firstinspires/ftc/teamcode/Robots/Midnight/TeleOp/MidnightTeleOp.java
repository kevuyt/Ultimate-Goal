package org.firstinspires.ftc.teamcode.Robots.Midnight.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Midnight;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 2020-01-17.
 * Project: MasqLib
 */
@TeleOp(name = "MidnightTeleOp", group = "ZZZ")
public class MidnightTeleOp extends MasqLinearOpMode {
    private Midnight robot = new Midnight();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeTeleop();

        while(!opModeIsActive()) {
            dash.create("Heading: ", robot.tracker.getHeading());
            dash.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            robot.NFS(controller1);
            robot.foundationHook.DriverControl(controller1);
        }
    }
}
