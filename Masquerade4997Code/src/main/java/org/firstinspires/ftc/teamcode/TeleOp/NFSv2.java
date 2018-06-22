package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Robot;

import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 6/2/18.
 * Project: MasqLib
 */

@TeleOp(name = "NFSV2", group = "NFS")
public class NFSv2 extends MasqLinearOpMode implements Constants {
    private Robot robot = new Robot();
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("Init");
            dash.update();
        }
        waitForStart();
        robot.leftMotor.setClosedLoop(true);
        robot.rightMotor.setClosedLoop(true);
        robot.leftMotor.setKp(0.01);
        robot.rightMotor.setKp(0.01);
        while (opModeIsActive()) {
            float move = controller1.leftStickY();
            float turn = controller1.rightStickX();
            double left = move + turn;
            double right = move - turn;
            if(MasqUtils.max(left, right) > 1) {
                left /= left;
                right /= left;
            }
            robot.leftMotor.setVelocity(-left);
            robot.rightMotor.setVelocity(-right);
        }
    }
}