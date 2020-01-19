package org.firstinspires.ftc.teamcode.Robots.MarkOne.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqResources.MasqUtils;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/14/2019
 */
@TeleOp(name = "RobotTeleOp", group = "MarkOne")
public class RobotTeleOp extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeTeleop();

        double prevGrabber = 1;
        double prevRotater = 0;
        double prevCapper = 0;

        while(!opModeIsActive()) {
            dash.create("Heading: ", robot.tracker.getHeading());
            dash.update();
        }

        waitForStart();

        robot.blockPusher.setPosition(1);
        double prevPusher = 1;


        while(opModeIsActive()) {
            if (controller1.rightBumper() || controller1.leftBumper())
                robot.MECH(controller1,0.5, 0.2);
            else robot.MECH(controller1,1, 0.4);

            if (controller1.leftTriggerPressed()) robot.intake.setVelocity(-1);
            else if (controller1.rightTriggerPressed()) robot.intake.setVelocity(1);
            else robot.intake.setVelocity(0);
            if (controller2.rightTriggerPressed()) robot.lift.setVelocity(0.5*controller2.leftStickY());
            else robot.lift.setVelocity(controller2.leftStickY());

            robot.tapeMeasure.setPower(controller2.rightStickY());

            MasqUtils.toggle(controller2.yOnPress(), robot.blockRotater, prevRotater);
            MasqUtils.toggle(controller2.xOnPress(), robot.blockGrabber, prevGrabber);
            MasqUtils.toggle(controller2.aOnPress(), robot.blockPusher,prevPusher);
            MasqUtils.toggle(controller2.dPadUpOnPress(), robot.capper, prevCapper);

            robot.foundationHook.DriverControl(controller1);

            robot.sideGrabber.teleReset();

            prevGrabber = robot.blockGrabber.getPosition();
            prevPusher = robot.blockPusher.getPosition();
            prevRotater = robot.blockRotater.getPosition();
            prevCapper = robot.capper.getPosition();

            dash.create("X: ",robot.tracker.getGlobalX());
            dash.create("Y: ",robot.tracker.getGlobalY());
            dash.create("Raw X: ",robot.tapeMeasure.getCurrentPosition());
            dash.create("Raw YL: ",robot.intake.motor2.getCurrentPosition());
            dash.create("Raw YR: ", robot.intake.motor1.getCurrentPosition());
            dash.create("XR stick: ", controller1.rightStickX());
            robot.tracker.updateSystem();
            dash.update();

            controller1.update();
            controller2.update();
        }
    }
}