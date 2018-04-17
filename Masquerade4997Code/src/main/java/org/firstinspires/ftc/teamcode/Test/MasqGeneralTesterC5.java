package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqWrappers.MasqLinearOpMode;

import static org.firstinspires.ftc.teamcode.AutonomusWorlds.Constants.ROTATOR_RED_NOT_SEEN;
import static org.firstinspires.ftc.teamcode.AutonomusWorlds.Constants.ROTATOR_RED_SEEN;
import static org.firstinspires.ftc.teamcode.TeleOp.Constants.JEWEL_RED_IN;
import static org.firstinspires.ftc.teamcode.TeleOp.Constants.JEWEL_RED_OUT;

/**
 * Created by Archish on 4/16/18.
 */
@Autonomous(name = "MasqGeneralTesterC5: TeleOp Toggle, Blue Jewel, Continuous t update", group = "T")
public class MasqGeneralTesterC5 extends MasqLinearOpMode {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        dash.startUpdate();
        int count = 0;
        while (!opModeIsActive()) {
            boolean pressed = false, released = false;
            if (controller1.a()) {
                released = false;
                pressed = true;
            }
            if (!controller1.a()) released = true;
            if (pressed && released) count++;
            dash.create("Count: ", count);
        }
        waitForStart();
        robot.jewelArmRed.setPosition(JEWEL_RED_OUT);
        robot.sleep(1000);
        if (robot.jewelColorRed.isBlue()) robot.redRotator.setPosition(ROTATOR_RED_NOT_SEEN);
        else robot.redRotator.setPosition(ROTATOR_RED_SEEN);
        robot.sleep(250);
        robot.jewelArmRed.setPosition(JEWEL_RED_IN);
        robot.sleep(100);
    }
}