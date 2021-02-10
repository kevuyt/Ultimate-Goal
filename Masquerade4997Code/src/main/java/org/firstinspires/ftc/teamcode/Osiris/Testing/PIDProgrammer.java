package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 2/9/2021
 */
@TeleOp(name = "PIDProgrammer", group = "Test")
public class PIDProgrammer  extends MasqLinearOpMode {
    private Osiris robot = new Osiris();
    double kp = 0;

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        dash.create("Hello, this is the PID Programmer");
        dash.update();

        waitForStart();

        while(opModeIsActive()) {
            if (controller1.a()) kp += 1e-10;
            else if (controller1.b()) kp -= 1e-10;

            kp = Range.clip(kp, 0,1);

            robot.MECH();

            robot.intake.setVelocity(controller1.rightTrigger()-controller1.leftTrigger());

            if(controller1.y()) {
                robot.shooter.setVelocity(1);
                robot.hopper.setPosition(1);
            }
            else {
                robot.shooter.setVelocity(0);
                robot.hopper.setPosition(0);
            }

            if(controller1.dPadUp()) robot.flicker.setPosition(1);
            else robot.flicker.setPosition(0);

            robot.shooter.setKp(kp);
            dash.create( " Shooter kP (+A, -B): ", kp);


            dash.update();
        }
    }
}
