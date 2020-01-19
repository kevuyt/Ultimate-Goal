package org.firstinspires.ftc.teamcode.Robots.Midnight.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Midnight;

import Library4997.MasqWrappers.MasqLinearOpMode;

import static org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Constants.CAP_DOWN;
import static org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Constants.CAP_UP;
import static org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Constants.GRAB;
import static org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Constants.GRAB_MID;
import static org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Constants.PIVOT_DOWN;
import static org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Constants.PIVOT_UP;
import static org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Constants.RELEASE;

/**
 * Created by Archishmaan Peyyety on 2020-01-17.
 * Project: MasqLib
 */
@TeleOp(name = "MidnightTeleOp", group = "ZZZ")
@Disabled

public class MidnightTeleOp extends MasqLinearOpMode {
    private Midnight robot = new Midnight();
    private double pivotPosition, grabPosition, capstonePos;
    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);
        robot.initializeTeleop();
        while(!opModeIsActive()) {
            dash.create("Grab Position: ", robot.grabber.getPosition());
            dash.create("Pivot Position: ", robot.pivot.getPosition());
            dash.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            robot.NFS(controller1);
            // Button: A
            robot.foundationHook.DriverControl(controller1);
            //
            if (controller1.leftBumper()) pivotPosition = PIVOT_UP;
            else pivotPosition = PIVOT_DOWN;

            if (controller1.rightBumper()) grabPosition = GRAB;
            else if (controller1.a()) grabPosition = GRAB_MID;
            else grabPosition = RELEASE;

            if (controller1.y()) capstonePos = CAP_DOWN;
            else capstonePos = CAP_UP;

            dash.create("Grab Position: ", grabPosition);
            dash.create("Pivot Position: ", pivotPosition);
            dash.create("Capstone Position: ", capstonePos);
            dash.update();
            robot.grabber.setPosition(grabPosition);
            robot.pivot.setPosition(pivotPosition);
            robot.capstone.setPosition(capstonePos);
        }
    }
}
