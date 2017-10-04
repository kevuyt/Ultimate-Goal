package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;

/**
 * Created by Archish on 10/4/17.
 */
@TeleOp (name = "Test Teleop")
public class TestTeleOp extends OpMode {
    Hardware robot = new Hardware();
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.left.setPosition((gamepad1.left_stick_x/2));
    }
}
