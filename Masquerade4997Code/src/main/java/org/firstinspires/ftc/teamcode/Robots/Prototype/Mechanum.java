package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/14/2019
 */
@TeleOp(name = "Mechanum", group = "Prototype")
public class Mechanum extends MasqLinearOpMode {
    private PrototypeRobot robot = new PrototypeRobot();

    @Override
    public void runLinearOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.blockGrabber.setPosition(1);
        double prevGrabber = 1;

        robot.blockPusher.setPosition(0);
        double prevPusher;

        robot.blockRotater.setPosition(0);
        double prevRotater = 0;

        robot.foundationHook.setPosition(1);
        double prevHook = 1;

        robot.initializeTeleop();
        robot.driveTrain.setClosedLoop(true);
        robot.lift.setClosedLoop(true);
        robot.lift.setKp(0.001);

        robot.driveTrain.resetEncoders();

        while(!opModeIsActive()) {

            dash.create("Hello ");
            dash.update();
        }


        waitForStart();

        robot.blockPusher.setPosition(1);
        prevPusher = 1;

        boolean prevX = false;
        boolean prevY = false;

        while(opModeIsActive()) {
            controller1.toggle(controller1.x(), controller1.y(), prevX, prevY, () -> {
                robot.multiplySpeedMultiplier(0.5);
                robot.multiplyTurnMultiplier(0.5);
            }, () -> {
                robot.multiplySpeedMultiplier(2);
                robot.multiplyTurnMultiplier(2);
            });
            robot.MECH(controller1);

            if (controller1.leftTriggerPressed()) robot.intake.setVelocity(-0.8);
            else if (controller1.rightTriggerPressed()) robot.intake.setVelocity(0.8);
            else robot.intake.setVelocity(0);

            if (controller2.rightTriggerPressed()) robot.lift.setVelocity(1);
            else if (controller2.leftTriggerPressed() && robot.lift.encoder.getInches() > 1) robot.lift.setVelocity(-controller2.leftTrigger());
            else robot.lift.setVelocity(0);

            if (robot.lift.encoder.getInches() > 5) controller2.toggle(controller2.yOnPress(), robot.blockRotater, prevRotater);
            controller2.toggle(controller2.xOnPress(), robot.blockGrabber, prevGrabber);
            controller2.toggle(controller2.aOnPress(), robot.blockPusher,prevPusher);
            controller1.toggle(controller1.bOnPress(),robot.foundationHook, prevHook);

            prevGrabber = robot.blockGrabber.getPosition();
            prevPusher = robot.blockPusher.getPosition();
            prevRotater = robot.blockRotater.getPosition();
            prevHook = robot.foundationHook.getPosition();

            dash.create("Speed: ", robot.speedMultiplier);
            dash.create("Turn: ", robot.turnMultiplier);
            dash.update();

            prevX = controller1.x();
            prevY = controller1.y();

            controller1.update();
            controller2.update();


        }
    }
}