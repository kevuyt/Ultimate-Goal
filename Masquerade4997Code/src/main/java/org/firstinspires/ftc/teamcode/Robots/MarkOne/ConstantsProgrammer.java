package org.firstinspires.ftc.teamcode.Robots.MarkOne;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 10/4/2019
 */
@TeleOp(name = "ConstantsProgrammer", group= "Prototype")
public class ConstantsProgrammer extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    private double blockGrabberPosition = 0;
    private double blockRotaterPosition = 0;
    private double blockPusherPosition = 0;

    @Override
    public void runLinearOpMode() throws InterruptedException{
        robot.mapHardware(hardwareMap);
        robot.blockGrabber.setPosition(0);
        robot.blockPusher.setPosition(0);
        robot.blockRotater.setPosition(0);

        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            if (controller1.aOnPress()) blockGrabberPosition += 0.1;
            if (controller1.bOnPress()) blockGrabberPosition -= 0.1;
            if (controller1.xOnPress()) blockRotaterPosition += 0.01;
            if (controller1.yOnPress()) blockRotaterPosition -= 0.01;
            if (controller1.leftTriggerOnPress()) blockPusherPosition += 0.1;
            if (controller1.rightTriggerOnPress()) blockPusherPosition -= 0.1;

            robot.blockGrabber.setPosition(blockGrabberPosition);
            robot.blockRotater.setPosition(blockRotaterPosition);
            robot.blockPusher.setPosition(blockPusherPosition);

            dash.create("Grabber Position: ", blockGrabberPosition);
            dash.create("Rotater Position: ", blockRotaterPosition);
            dash.create("Pusher Position: ", blockPusherPosition);
            dash.update();

            controller1.update();
        }
    }
}