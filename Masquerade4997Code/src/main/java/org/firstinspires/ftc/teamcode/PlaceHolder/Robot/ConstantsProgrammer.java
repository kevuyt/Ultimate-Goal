package org.firstinspires.ftc.teamcode.PlaceHolder.Robot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 2020-01-12.
 * Project: MasqLib
 */
@TeleOp(name = "ConstantsProgrammer", group = "PlaceHolder")
public class ConstantsProgrammer extends MasqLinearOpMode {
    public PlaceHolder robot = new PlaceHolder();
    private double claw, rotater;

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);

        while (!opModeIsActive()) {
            dash.create("Hello, this is a constants programmer.");
            dash.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            if (controller1.a()) claw += 0.001;
            else if (controller1.b()) claw -= 0.001;

            if (controller1.y()) rotater += 0.001;
            else if (controller1.x()) rotater -= 0.001;

            claw = Range.clip(claw, 0,1);
            rotater = Range.clip(rotater, 0,1);

            robot.claw.claw.setPosition(claw);
            robot.claw.rotater.setPosition(rotater);
            dash.create("Claw (+A, -B): ", claw);
            dash.create("Rotater (+Y, -X): ", rotater);
            dash.update();
        }
    }
}
