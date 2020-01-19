package org.firstinspires.ftc.teamcode.Robots.Midnight.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Midnight;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 2020-01-17.
 * Project: MasqLib
 */
@TeleOp(name = "ConstantsProgrammer", group = "ZZZ")
@Disabled

public class ConstantsProgrammer extends MasqLinearOpMode {
    public Midnight robot = new Midnight();
    private double leftHookPos, rightHookPos, pivotPos, grabberPos, capstone;
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        while (!opModeIsActive()) {
            dash.create("Bruh moment");
            dash.update();
        }

        waitForStart();
        while (opModeIsActive()) {
            if (controller1.a()) leftHookPos += 0.001;
            else if (controller1.y()) leftHookPos -= 0.001;
            leftHookPos = Range.clip(leftHookPos,0,1);
            robot.foundationHook.leftHook.setPosition(leftHookPos);

            if (controller1.x()) rightHookPos += 0.001;
            else if (controller1.b()) rightHookPos -= 0.001;
            rightHookPos = Range.clip(rightHookPos,0,1);
            robot.foundationHook.rightHook.setPosition(rightHookPos);

            if (controller1.rightBumper()) grabberPos += 0.001;
            else if (controller1.rightTriggerPressed()) grabberPos -= 0.001;
            grabberPos = Range.clip(grabberPos,0,1);
            robot.grabber.setPosition(grabberPos);

            if (controller1.leftBumper()) pivotPos += 0.001;
            else if (controller1.leftTriggerPressed()) pivotPos -= 0.001;
            pivotPos = Range.clip(pivotPos,0,1);
            robot.pivot.setPosition(pivotPos);

            if (controller2.a()) capstone += 0.001;
            else if (controller2.y()) capstone -= 0.001;
            capstone = Range.clip(capstone,0,1);
            robot.capstone.setPosition(capstone);

            dash.create("Left Hook Position: ", leftHookPos);
            dash.create("Right Hook Position: ", rightHookPos);
            dash.create("Grabber Position: ", grabberPos);
            dash.create("Pivot Position: ", pivotPos);
            dash.create("Capstone Position: ", capstone);
            dash.update();
        }
    }
}
