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
            dash.create("Manual Inches: ",robot.intake.motor2.getCurrentPosition() /
                    (1440 / (2 * Math.PI)));
            dash.update();
        }

        waitForStart();
        robot.blockPusher.setPosition(1);
        double prevPusher = 1;


        while(opModeIsActive()) {
            if (controller1.rightBumper() || controller1.leftBumper())
                robot.MECH(controller1,0.5, 0.15);
            else robot.MECH(controller1,1, 0.5);

            if (controller1.leftTriggerPressed()) robot.intake.setVelocity(-1);
            else if (controller1.rightTriggerPressed()) robot.intake.setVelocity(1);
            else robot.intake.setVelocity(0);

            robot.lift.setVelocity(controller2.leftStickY());
            MasqUtils.toggle(controller2.yOnPress(), robot.blockRotater, prevRotater);
            MasqUtils.toggle(controller2.xOnPress(), robot.blockGrabber, prevGrabber);
            MasqUtils.toggle(controller2.aOnPress(), robot.blockPusher,prevPusher);
            MasqUtils.toggle(controller2.dPadUpOnPress(), robot.capper, prevCapper);

            robot.foundationHook.DriverControl(controller1);

            robot.sideGrabber.leftClose();
            robot.sideGrabber.rightClose();
            robot.sideGrabber.rightUp();
            robot.sideGrabber.leftUp();

            prevGrabber = robot.blockGrabber.getPosition();
            prevPusher = robot.blockPusher.getPosition();
            prevRotater = robot.blockRotater.getPosition();
            prevCapper = robot.capper.getPosition();

            dash.create("X: ",robot.tracker.getGlobalX());
            dash.create("Y: ",robot.tracker.getGlobalY());
            dash.create("Raw X: ",robot.X.getCurrentPosition());
            dash.create("Raw YL: ",robot.intake.motor2.getCurrentPosition());
            dash.create("Raw YR: ", robot.intake.motor1.getCurrentPosition());
            dash.create("XR stick: ", controller1.rightStickX());
            dash.update();

            controller1.update();
            controller2.update();
        }
    }
}