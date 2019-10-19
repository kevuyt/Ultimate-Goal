package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

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

        robot.driveTrain.setClosedLoop(true);
        while(!opModeIsActive()) {
            dash.create("Big Brain Time");
            dash.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            robot.MECH(controller1);

            if (controller1.leftBumper()) robot.intake.setVelocity(-1);
            else if (controller1.rightTriggerPressed()) {
                robot.intake.setVelocity(1);
                robot.blockPusher.setPosition(1);
                robot.blockGrabber.setPosition(1);
            }
            else if (controller1.rightBumper()) robot.intake.setVelocity(-1);
            else if (controller1.leftTriggerPressed()) {
                robot.intake.setVelocity(1);
                robot.blockPusher.setPosition(1);
                robot.blockGrabber.setPosition(1);
            }
            else robot.intake.setVelocity(0);

            if (controller2.leftTriggerPressed()) robot.lift.setVelocity(1);
            else if (controller2.rightTriggerPressed()) robot.lift.setVelocity(-controller2.rightTrigger());
            else robot.lift.setVelocity(0);

            if (robot.lift.encoder.getInches() > 1) controller2.toggle(controller2.aOnPress(), robot.blockRotater, prevRotater);
                controller2.toggle(controller2.yOnPress(), robot.blockGrabber,prevGrabber);
                controller2.toggle(controller2.xOnPress(), robot.blockPusher,prevPusher);

            prevGrabber = robot.blockGrabber.getPosition();
            prevPusher = robot.blockPusher.getPosition();
            prevRotater = robot.blockRotater.getPosition();
            dash.create("Motor4 Power", robot.driveTrain.rightDrive.motor2.getVelocity(), robot.driveTrain.rightDrive.motor2.getPower());
            dash.create("Proportional", robot.driveTrain.rightDrive.motor2.P);
            dash.create("Integral", robot.driveTrain.rightDrive.motor2.I);
            dash.create("Derivative", robot.driveTrain.rightDrive.motor2.D);
            dash.update();
            controller1.update();
            controller2.update();
        }
    }
}