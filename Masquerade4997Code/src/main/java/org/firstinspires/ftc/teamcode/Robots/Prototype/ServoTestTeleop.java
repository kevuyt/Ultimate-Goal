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
     while (!opModeIsActive()) {
         robot.mapHardware(hardwareMap);
     }
     waitForStart();
     while (opModeIsActive()) {
         if (controller1.aOnPress()) blockGrabberPosition += 0.1;
         if (controller1.bOnPress()) blockGrabberPosition -= 0.1;
         if (controller1.xOnPress()) blockRotaterPosition += 0.1;
         if (controller1.yOnPress()) blockRotaterPosition -= 0.1;
         if (controller1.leftTriggerOnPress()) blockPusherPosition += 0.1;
         if (controller1.rightTriggerOnPress()) blockPusherPosition -= 0.1;
         robot.blockGrabber.setPosition(blockGrabberPosition);
         robot.blockRotater.setPosition(blockRotaterPosition);
         robot.blockPusher.setPosition(blockPusherPosition);

     }
    }
}
