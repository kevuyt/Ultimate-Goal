package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 10/5/17.
 */
@Autonomous(name = "Jewel Auto", group = "Autonomus")
public class JewelAuto extends MasqLinearOpMode implements Constants {
    boolean red;
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
                if (controller1.aOnPress() && !red) {
                    dash.clear();
                    dash.create("THIS IS RED");
                    red = true;
                    controller1.update();
                }
                else if (controller1.aOnPress() && red) {
                    dash.clear();
                    dash.create("THIS IS BLUE");
                    red = false;
                    controller1.update();
                }
                controller1.update();
                //dash.create(INIT_MESSAGE);
                dash.update();
            }
        waitForStart();
        robot.jewelArm.setPosition(JEWEL_OUT);
        MasqExternal.sleep(10000);
        if (!red) {
            if (robot.jewelColor.isRed()) {
                robot.drive(-30);
            } else {
                robot.drive(30);
            }
        } else {
            if (robot.jewelColor.isBlue()) {
                robot.drive(-30);
            }
            else {
                robot.drive(30);
            }
        }
        robot.jewelArm.setPosition(JEWEL_IN);
        MasqExternal.sleep(10000);
    }
}