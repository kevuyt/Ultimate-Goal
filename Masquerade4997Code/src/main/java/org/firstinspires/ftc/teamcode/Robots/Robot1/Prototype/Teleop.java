package org.firstinspires.ftc.teamcode.Robots.Robot1.Prototype;

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
        PrototypeRobot robot = new PrototypeRobot();

        while(!opModeIsActive()) {
            dash.create("Done lol");
        }

        waitForStart();

        while(opModeIsActive()) {
            robot.MECH(controller1);
            robot.Grabber.setPosition(grabberPosition);
            if (controller1.aOnPress()) grabberPosition+= 0.1;
            robot.Twister.setPosition(twisterPosition);
            if (controller1.bOnPress()) twisterPosition+= 0.1;
            robot.Intake.setPower(controller1.leftTrigger()-controller1.rightTrigger());
        }
    }
}
