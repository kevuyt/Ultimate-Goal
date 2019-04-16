package org.firstinspires.ftc.teamcode.Robots.Reserection.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Reserection.Resurrection;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/27/18.
 * Project: MasqLib
 */
@TeleOp(name = "MECH", group = "NFS")
public class MECH extends MasqLinearOpMode implements Constants {
    private Resurrection resurrection = new Resurrection();
    @Override
    public void runLinearOpMode()  {
        resurrection.setStartOpenCV(false);
        resurrection.mapHardware(hardwareMap);
        resurrection.initializeTeleop();
        resurrection.hang.setClosedLoop(true);
        resurrection.driveTrain.setClosedLoop(true);
        resurrection.particleDumper.setPosition(PARTICLE_DUMPER_IN);
        while (!opModeIsActive()) {
            dash.create("X: ", resurrection.tracker.getGlobalX());
            dash.create("Y: ", resurrection.tracker.getGlobalY());
            dash.create("H: ", resurrection.tracker.getHeading());
            resurrection.tracker.updateSystem();
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            resurrection.MECH(controller1);

            if (controller1.leftTriggerPressed()) resurrection.collector.setPower(.5);
            else if (controller1.leftBumper()) resurrection.collector.setPower(-.5);
            else resurrection.collector.setPower(0);

            if (controller2.b()) resurrection.particleDumper.setPosition(PARTICLE_DUMPER_OUT);
            else resurrection.particleDumper.setPosition(PARTICLE_DUMPER_IN);

            if (controller2.leftStickY() < 0 && resurrection.hangTopSwitch.isPressed()) resurrection.hang.setPower(-1);
            else if (controller2.leftStickY() > 0 && resurrection.hangBottomSwitch.isPressed()) resurrection.hang.setPower(1);
            else resurrection.hang.setBreakMode();

            resurrection.collectorDumper.DriverControl(controller1);
            resurrection.collectionLift.DriverControl(controller1);
            resurrection.scoreLift.DriverControl(controller2);

            resurrection.tracker.updateSystem();
            dash.update();
        }
    }
}
