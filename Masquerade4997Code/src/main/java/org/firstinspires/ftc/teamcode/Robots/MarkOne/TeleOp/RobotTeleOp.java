package org.firstinspires.ftc.teamcode.Robots.MarkOne.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqResources.MasqUtils;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/14/2019
 */
@TeleOp(name = "RobotTeleOp", group = "Prototype")
public class RobotTeleOp extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeTeleop();

        double prevGrabber = 1;
        double prevRotater = 0;
        double prevCapper = 0;

        robot.driveTrain.resetEncoders();

        dash.create("Hello ");
        dash.update();

        waitForStart();

        robot.blockPusher.setPosition(1);
        double prevPusher = 1;

        robot.foundationHook.mid();

        while(opModeIsActive()) {
            robot.MECH(controller1);

            if (controller1.rightBumper() || controller1.leftBumper()) {
                robot.setSpeedMultiplier(0.5);
                robot.setTurnMultiplier(0.35);
            }
            else {
                robot.setSpeedMultiplier(1);
                robot.setTurnMultiplier(0.7);
            }

            if (controller1.leftTriggerPressed()) robot.intake.setVelocity(-0.8);
            else if (controller1.rightTriggerPressed()) robot.intake.setVelocity(0.8);
            else robot.intake.setVelocity(0);

            if (controller2.rightTriggerPressed()) robot.lift.setVelocity(1);
            else if (controller2.leftTriggerPressed()) robot.lift.setVelocity(-controller2.leftTrigger());
            else robot.lift.setVelocity(0);

            if (Math.abs(robot.lift.encoder.getInches()) > 10) MasqUtils.toggle(controller2.yOnPress(), robot.blockRotater, prevRotater);
            MasqUtils.toggle(controller2.xOnPress(), robot.blockGrabber, prevGrabber);
            MasqUtils.toggle(controller2.aOnPress(), robot.blockPusher,prevPusher);
            MasqUtils.toggle(controller2.dPadUpOnPress(), robot.capper, prevCapper);

            robot.foundationHook.DriverControl(controller1);

            prevGrabber = robot.blockGrabber.getPosition();
            prevPusher = robot.blockPusher.getPosition();
            prevRotater = robot.blockRotater.getPosition();
            prevCapper = robot.capper.getPosition();

            dash.create("Lift: ", robot.lift.encoder.getInches());
            dash.update();

            controller1.update();
            controller2.update();
        }
    }
}