package org.firstinspires.ftc.teamcode.Robots.MarkOne.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 2020-01-12.
 * Project: MasqLib
 */
@TeleOp(name = "ConstantsProgrammer", group = "MarkOne")
@Disabled
public class ConstantsProgrammer extends MasqLinearOpMode {
    public MarkOne robot = new MarkOne();
    private double hookPos;
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        while (!opModeIsActive()) {
            dash.create("Bruh moment");
            dash.update();
        }

        waitForStart();
        while (opModeIsActive()) {
            if (controller1.a()) hookPos+=0.001;
            else if (controller1.b()) hookPos-=0.001;
            hookPos = Range.clip(hookPos,0,1);
            robot.foundationHook.rightHook.setPosition(1-hookPos);
            robot.foundationHook.leftHook.setPosition(hookPos);
            dash.create("Position: ", hookPos);
            dash.update();
        }
    }
}
