package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/14/2019
 */
@TeleOp(name = "PrototypeTeleopv2", group = "Prototype")
public class PrototypeTeleopv2 extends MasqLinearOpMode {
    private PrototypeRobot robot = new PrototypeRobot();

    @Override
    public void runLinearOpMode() {
        robot.mapHardware(hardwareMap);

        robot.blockGrabber.setPosition(1);
        double prevGrabber = 1;

        robot.blockPusher.setPosition(1);
        double prevPusher = 1;

        robot.blockRotater.setPosition(0);
        double prevRotater = 0;

        robot.foundationHook.setPosition(1);
        double prevHook = 1;
        robot.initializeTeleop();
        robot.driveTrain.setClosedLoop(true);
        while(!opModeIsActive()) {
            dash.create("Big Brain Time");
            dash.update();
        }

        while (!opModeIsActive()) {
            robot.driveTrain.resetEncoders();
        }

        waitForStart();

        while(opModeIsActive()) {
            if (controller1.xOnPress()) {
                robot.setVelocityMagnitude(0.5);
            }
            else if (controller1.yOnPress()) {
                robot.setVelocityMagnitude(1);
            }
            robot.MECH(controller1);

            if (controller1.leftTriggerPressed()) robot.intake.setVelocity(-0.7);
            else if (controller1.rightTriggerPressed()) robot.intake.setVelocity(0.7);
            else robot.intake.setVelocity(0);

            if (controller2.rightTriggerPressed()) robot.lift.setVelocity(1);
            else if (controller2.leftTriggerPressed()) robot.lift.setVelocity(-controller2.leftTrigger());
            else robot.lift.setVelocity(0);

            if (robot.lift.encoder.getInches() > 1) controller2.toggle(controller2.yOnPress(), robot.blockRotater, prevRotater);
            controller2.toggle(controller2.xOnPress(), robot.blockGrabber, prevGrabber, () -> robot.blockPusher.setPosition(1));
            controller2.toggle(controller2.aOnPress(), robot.blockPusher,prevPusher);
            controller1.toggle(controller1.bOnPress(),robot.foundationHook, prevHook);

            prevGrabber = robot.blockGrabber.getPosition();
            prevPusher = robot.blockPusher.getPosition();
            prevRotater = robot.blockRotater.getPosition();
            prevHook = robot.foundationHook.getPosition();

            dash.create("Power", robot.driveTrain.rightDrive.motor1.error, robot.driveTrain.rightDrive.motor2.error);
            dash.update();

            controller1.update();
            controller2.update();
        }
    }
}