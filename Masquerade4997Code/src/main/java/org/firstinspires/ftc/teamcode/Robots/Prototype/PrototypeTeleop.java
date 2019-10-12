package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/14/2019
 */
@TeleOp(name = "PrototypeTeleop", group = "Prototype")
public class PrototypeTeleop extends MasqLinearOpMode {
    private PrototypeRobot robot = new PrototypeRobot();
    @Override
    public void runLinearOpMode() {
        robot.mapHardware(hardwareMap);
        robot.blockGrabber.setPosition(1);
        double prevGrabber = 1;
        robot.blockPusher.setPosition(0);
        double prevPusher = 0;
        robot.blockRotater.setPosition(0);
        double prevRotater = 0;

        while(!opModeIsActive()) {
            dash.create("Big Brain Time");
            dash.update();
        }

        waitForStart();

        robot.blockPusher.setPosition(1);

        while(opModeIsActive()) {
            robot.MECH(controller1);

            if (controller1.rightTriggerPressed()) robot.intake.setVelocity(-1);
            else if (controller1.leftTriggerPressed()) robot.intake.setVelocity(1);
            else robot.intake.setVelocity(0);

            if (controller2.leftTriggerPressed()) robot.lift.setVelocity(1);
            else if (controller2.rightTriggerPressed()) robot.lift.setVelocity(-controller2.rightTrigger());
            else robot.lift.setVelocity(0);

            if (controller2.aOnPress() && robot.blockRotater.getPosition() == prevRotater) {
                if (robot.blockRotater.getPosition() == 0) {
                    robot.blockRotater.setPosition(1);
                }
                else if (robot.blockRotater.getPosition() == 1) {
                    robot.blockRotater.setPosition(0);
                }
            }

            if (controller2.yOnPress() && robot.blockGrabber.getPosition() == prevGrabber) {
                if (robot.blockGrabber.getPosition() == 1) {
                    robot.blockGrabber.setPosition(0);
                } else if (robot.blockGrabber.getPosition() == 0) {
                    robot.blockGrabber.setPosition(1);
                }
            }

            if (controller2.xOnPress() && robot.blockPusher.getPosition() == prevPusher){
                if (robot.blockPusher.getPosition() == 1) {
                    robot.blockPusher.setPosition(0);
                }
                else if (robot.blockPusher.getPosition() == 0) {
                    robot.blockPusher.setPosition(1);
                }
            }

            prevGrabber = robot.blockGrabber.getPosition();
            prevPusher = robot.blockPusher.getPosition();
            prevRotater = robot.blockRotater.getPosition();
            dash.create("Grabber: ", prevGrabber);
            dash.create("Pusher: ", prevPusher);
            dash.create("Rotater: ", prevRotater);
            dash.update();
            controller1.update();
            controller2.update();
        }
    }
}
