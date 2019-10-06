package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqResources.MasqUtils;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/14/2019
 */
@TeleOp(name = "PrototypeTeleop", group = "Prototype")
public class PrototypeTeleop extends MasqLinearOpMode {
    private PrototypeRobot robot = new PrototypeRobot();
    @Override
    public void runLinearOpMode() {
        robot.mapHardware(hardwareMap);
        robot.blockGrabber.setPosition(0.5);
        robot.blockPusher.setPosition(0);
        robot.blockRotater.setPosition(0);
        while(!opModeIsActive()) {
            dash.create("Big Brain Time");
            dash.update();
        }

        waitForStart();
        robot.blockPusher.setPosition(0.4);
        while(opModeIsActive()) {
            robot.MECH(controller1);
            if (controller1.rightTriggerPressed()) robot.intakeRight.setPower(-1);
            else if (controller1.rightBumper()) robot.intakeRight.setPower(1);
            else robot.intakeRight.setPower(0);
            if (controller1.leftTriggerPressed()) robot.intakeLeft.setPower(1);
            else if (controller1.leftBumper()) robot.intakeLeft.setPower(-1);
            else robot.intakeLeft.setPower(0);
            robot.lift.setPower(controller2.leftTrigger()-0.3*controller2.rightTrigger());
            if (controller2.a() && robot.blockRotater.getPosition() == 0) robot.blockRotater.setPosition(1);
            else if (controller2.a() && robot.blockRotater.getPosition() == 1) robot.blockRotater.setPosition(0);
            if (controller2.y() && robot.blockGrabber.getPosition() == 0.5) robot.blockGrabber.setPosition(0);
            else if (controller2.y() && robot.blockGrabber.getPosition() == 0) robot.blockGrabber.setPosition(0.5);
            if (controller2.x() && robot.blockPusher.getPosition() == 0) robot.blockPusher.setPosition(0.4);
            else if (controller2.x() && robot.blockPusher.getPosition() == 0.4) robot.blockPusher.setPosition(0);
            if (controller2.rightTriggerPressed()) {
                robot.blockPusher.setPosition(0);
                MasqUtils.sleep(1.0);
                robot.blockGrabber.setPosition(0);
                robot.blockPusher.setPosition(0.4);
            }
        }
    }
}
