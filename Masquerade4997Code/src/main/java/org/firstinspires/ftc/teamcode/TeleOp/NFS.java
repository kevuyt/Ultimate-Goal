package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Robot;

import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 6/2/18.
 * Project: MasqLib
 */

@TeleOp(name = "NFS", group = "NFS")
public class NFS extends MasqLinearOpMode implements Constants {
    private Robot robot = new Robot();
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("Init");
            robot.yTranslator.setVelocity(1);
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            float yTranslation = controller1.leftStickX();
            float move = controller1.leftStickY();
            float turn = controller1.rightStickX();
            double left = move + turn;
            double right = move - turn;
            if(MasqUtils.max(left, right) > 1) {
                left /= left;
                right /= left;
            }
            robot.leftMotor.setPower(-left);
            robot.rightMotor.setPower(-right);
            //robot.yTranslator.setPower(yTranslation);
        }
    }
}
