package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 10/4/2019
 */
@TeleOp(name = "ServoTestTeleop", group= "Prototype")
public class ServoTestTeleop extends MasqLinearOpMode {
    private PrototypeRobot robot = new PrototypeRobot();
    private double blockGrabberPosition = 0;
    private double blockRotaterPosition = 0;
    private double blockPusherPosition = 0;

    @Override
    public void runLinearOpMode() {
        robot.mapHardware(hardwareMap);
        robot.blockGrabber.setPosition(0);
        robot.blockPusher.setPosition(0);
        robot.blockRotater.setPosition(0);
        while (!opModeIsActive()) {
            dash.create("Creeper");
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            if (controller1.a() && !(blockGrabberPosition > 1)) blockGrabberPosition += 0.1;
            if (controller1.b() && !(blockGrabberPosition < 0)) blockGrabberPosition -= 0.1;
            if (controller1.x() && !(blockRotaterPosition > 1)) blockRotaterPosition += 0.1;
            if (controller1.y() && !(blockRotaterPosition < 1)) blockRotaterPosition -= 0.1;
            if (controller1.leftTriggerPressed() && !(blockPusherPosition > 1)) blockPusherPosition += 0.1;
            if (controller1.rightTriggerPressed() && !(blockPusherPosition < 1)) blockPusherPosition -= 0.1;
            robot.blockGrabber.setPosition(blockGrabberPosition);
            robot.blockRotater.setPosition(blockRotaterPosition);
            robot.blockPusher.setPosition(blockPusherPosition);
            dash.create("Grabber Position: ", blockGrabberPosition, robot.blockGrabber.getPosition());
            dash.create("Rotater Position: ", blockRotaterPosition, robot.blockRotater.getPosition());
            dash.create("Pusher Position: ", blockPusherPosition, robot.blockPusher.getPosition());
            dash.update();
            controller1.update();

        }
    }
}
