package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        while(!opModeIsActive()) {
            dash.create("Big Brain Time");
            dash.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            robot.MECH(controller1);
            if (controller1.rightTriggerPressed()) robot.intakeRight.setPower(-1);
            else if (controller1.rightBumper()) robot.intakeRight.setPower(1);
            else robot.intakeRight.setPower(0);
            if (controller1.leftTriggerPressed()) robot.intakeLeft.setPower(1);
            else if (controller1.leftBumper()) robot.intakeLeft.setPower(-1);
            else robot.intakeLeft.setPower(0);
            robot.lift.setPower(controller2.leftTrigger()-0.3*controller2.rightTrigger());
            if (controller2.a()) robot.blockRotater.setPosition(1);
            else robot.blockRotater.setPosition(0);
            if (controller2.b()) robot.blockGrabber.setPosition(1);
            else robot.blockGrabber.setPosition(0);
            if (controller2.x()) robot.blockPusher.setPosition(1);
            else robot.blockPusher.setPosition(0);
        }
    }
}
