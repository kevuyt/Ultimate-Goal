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

        robot.initializeTeleop();
        robot.driveTrain.setClosedLoop(true);
        robot.lift.setClosedLoop(true);
        robot.lift.setKp(0.001);

        robot.resetServos();

        double prevGrabber = 1;
        double prevPusher;
        double prevRotater = 0;
        double prevSide = 0;

        robot.driveTrain.resetEncoders();

        while(!opModeIsActive()) {

            dash.create("Hello ");
            dash.update();
        }

        waitForStart();

        robot.blockPusher.setPosition(1);
        prevPusher = 1;
        robot.midFoundationHook();

        while(opModeIsActive()) {
            robot.MECH(controller1);

            if (controller1.rightBumper() || controller1.leftBumper()) robot.setMultipliers(0.5);
            else {
                robot.setSpeedMultiplier(1);
                robot.setTurnMultiplier(0.8);
            }

            if (controller1.leftTriggerPressed()) robot.intake.setVelocity(-0.8);
            else if (controller1.rightTriggerPressed()) robot.intake.setVelocity(0.8);
            else robot.intake.setVelocity(0);

            if (controller2.rightTriggerPressed()) robot.lift.setVelocity(1);
            else if (controller2.leftTriggerPressed()) robot.lift.setVelocity(-controller2.leftTrigger());
            else robot.lift.setVelocity(0);

            if (robot.lift.encoder.getInches() > 24) controller2.toggle(controller2.yOnPress(), robot.blockRotater, prevRotater);
            controller2.toggle(controller2.xOnPress(), robot.blockGrabber, prevGrabber);
            controller2.toggle(controller2.aOnPress(), robot.blockPusher,prevPusher);
            if (controller1.b()) robot.lowerFoundationHook();
            else if (controller1.x()) robot.raiseFoundationHook();
            else robot.midFoundationHook();
            controller1.toggle(controller1.xOnPress(), robot.sideGrabber, prevSide);

            prevGrabber = robot.blockGrabber.getPosition();
            prevPusher = robot.blockPusher.getPosition();
            prevRotater = robot.blockRotater.getPosition();
            prevSide = robot.sideGrabber.getPosition();

            dash.create("Lift Position: ", robot.lift.encoder.getInches());
            dash.update();

            controller1.update();
            controller2.update();


        }
    }
}