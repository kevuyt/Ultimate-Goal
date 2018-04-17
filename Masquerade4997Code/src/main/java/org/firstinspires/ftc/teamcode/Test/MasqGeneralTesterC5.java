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
@Autonomous(name = "MasqGeneralTesterC5: TeleOp Toggle, Blue Jewel", group = "T")
public class MasqGeneralTesterC5 extends MasqLinearOpMode {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        boolean toggle = false;
        int count = 0;
        while (!opModeIsActive()) {
            if (controller1.a()) toggle = true;
            if (toggle) {
                count++;
                toggle = false;
            }
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