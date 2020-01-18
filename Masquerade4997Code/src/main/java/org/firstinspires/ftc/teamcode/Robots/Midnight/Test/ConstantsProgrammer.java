package org.firstinspires.ftc.teamcode.Robots.Midnight.Test;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Midnight;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 2020-01-17.
 * Project: MasqLib
 */
public class ConstantsProgrammer extends MasqLinearOpMode {
    public Midnight robot = new Midnight();
    private double leftHookPos, rightHookPos;
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

            dash.create("Left Hook Position: ", leftHookPos);
            dash.create("Right Hook Position: ", rightHookPos);
            dash.update();
        }
    }
}
