package org.firstinspires.ftc.teamcode.Robots.Prototype;

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
            robot.gripper.setPosition(grabberPosition);
            if (controller1.aOnPress()) grabberPosition+= 0.1;
            robot.blockRotater.setPosition(twisterPosition);
            if (controller1.bOnPress()) twisterPosition+= 0.1;
            robot.intake.setPower(controller1.leftTrigger()-controller1.rightTrigger());
            dash.create("gripper Position: ", grabberPosition);
            dash.create("blockRotater Position: ", twisterPosition);
            dash.update();
        }
    }
}
