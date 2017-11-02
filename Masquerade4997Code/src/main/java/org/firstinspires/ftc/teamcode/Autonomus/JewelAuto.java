package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 10/5/17.
 */
@Autonomous(name = "DRIVE_ENCODER PID TEST", group = "Autonomus")
public class JewelAuto extends MasqLinearOpMode implements Constants {
    Direction direction;
    boolean red;
    public void runLinearOpMode() throws InterruptedException {
        direction = Direction.BACKWARD;
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            while (!opModeIsActive()) {
                if (controller1.aOnPress() && direction != Direction.BACKWARD && !red) {
                    dash.clear();
                    dash.create("THIS WILL GO BACKWARD, AND IS RED");
                    red = true;
                    direction = Direction.BACKWARD;
                    controller1.update();
                }
                else if (controller1.aOnPress() && direction != Direction.BACKWARD && red) {
                    dash.clear();
                    dash.create("THIS WILL GO BACKWARD, AND IS BLUE");
                    red = false;
                    direction = Direction.FORWARD;
                    controller1.update();
                } else if (controller1.aOnPress() && direction != Direction.FORWARD && !red) {
                    dash.clear();
                    dash.create("THIS WILL GO FOREWORD, AND IS RED");
                    red = true;
                    direction = Direction.FORWARD;
                    controller1.update();
                } else if (controller1.aOnPress() && direction != Direction.FORWARD && red) {
                    dash.clear();
                    dash.create("THIS WILL GO FOREWORD, AND IS BLUE");
                    red = false;
                    direction = Direction.FORWARD;
                    controller1.update();
                }
                controller1.update();
                //dash.create(INIT_MESSAGE);
                dash.update();
            }
        }
        waitForStart();
        robot.jewelArm.setPosition(JEWEL_OUT);
        MasqExternal.sleep(100);
        if (red) {
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
    }
}