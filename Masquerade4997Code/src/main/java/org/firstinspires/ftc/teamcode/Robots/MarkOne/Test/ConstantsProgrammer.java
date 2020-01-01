package org.firstinspires.ftc.teamcode.Robots.MarkOne.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 10/4/2019
 */
@TeleOp(name = "ConstantsProgrammer", group= "MarkOne")
public class ConstantsProgrammer extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    private double hookPos, autoRotPos, autoGrabPos;

    @Override
    public void runLinearOpMode() throws InterruptedException{
        robot.init(hardwareMap);

        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            if (controller1.a()) hookPos += 0.1;
            if (controller1.b()) hookPos -= 0.1;
            if (controller1.x()) autoRotPos += 0.1;
            if (controller1.y()) autoRotPos -= 0.1;
            if (controller1.dPadUp()) autoGrabPos += 0.1;
            if (controller1.dPadDown()) autoGrabPos -= 0.1;

            hookPos = Range.clip(hookPos,0,1);
            autoGrabPos = Range.clip(autoGrabPos, 0,1);
            autoRotPos = Range.clip(autoRotPos,0,1);

            dash.create("Hook Position: ", hookPos);
            dash.create("Auto Rotater Position: ", autoRotPos);
            dash.create("Auto Grab Position: ", autoGrabPos);
            dash.update();

            robot.foundationHook.leftHook.setPosition(hookPos);
            robot.sideGrabberLeft.sideGrabber.servo1.setPosition(autoRotPos);
            robot.sideGrabberLeft.sideGrabber.servo2.setPosition(autoGrabPos);
            sleep(1);
            controller1.update();
        }
    }
}