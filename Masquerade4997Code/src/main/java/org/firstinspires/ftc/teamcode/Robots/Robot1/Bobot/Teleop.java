package org.firstinspires.ftc.teamcode.Robots.Robot1.Bobot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/14/2019
 */
@TeleOp(name = "Teleop")
public class Teleop extends MasqLinearOpMode {
    private double grabberPosition = 0;
    private double twisterPosition = 0;
    @Override
    public void runLinearOpMode() throws InterruptedException {
        BobotRobot robot = new BobotRobot();

        while(!opModeIsActive()) {
            dash.create("Done lol");
        }

        waitForStart();

        while(opModeIsActive()) {
            robot.MECH(controller1);
            robot.Grabber.setPosition(grabberPosition);
            if (controller1.a()) grabberPosition+= 0.1;
            robot.Twister.setPosition(twisterPosition);
            if (controller1.b()) twisterPosition+= 0.1;
            robot.Gripper.setPower(controller1.leftTrigger()-controller1.rightTrigger());
        }
    }
}
