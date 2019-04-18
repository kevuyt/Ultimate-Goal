package org.firstinspires.ftc.teamcode.Robots.Reserection.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Reserection.Resurrection;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/27/18.
 * Project: MasqLib
 */
@TeleOp(name = "MECH", group = "NFS")
@Disabled
public class MECH extends MasqLinearOpMode implements Constants {
    private Resurrection resurrection = new Resurrection();
    @Override
    public void runLinearOpMode()  {
        resurrection.setStartOpenCV(false);
        resurrection.mapHardware(hardwareMap);
        resurrection.initializeTeleop();
        resurrection.hang.setClosedLoop(true);
        resurrection.driveTrain.setClosedLoop(true);
        resurrection.particleDumper.setPosition(PARTICLE_DUMPER_OUT);
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

            if (controller1.dPadUp()) resurrection.collectorDumper.setPower(.7);
            else if (controller1.dPadDown()) {
                if (!resurrection.collectionLiftSwitch.isPressed()) resurrection.collectionLift.lift.setPower(-1);
                resurrection.collectorDumper.setPower(-.7);
            }
            else resurrection.collectorDumper.setPower(0);

            if (controller2.b()) resurrection.particleDumper.setPosition(PARTICLE_DUMPER_OUT);
            else if (controller2.rightTriggerPressed()) resurrection.particleDumper.setPosition(PARTICLE_DUMPER_PARALLEL);
            else resurrection.particleDumper.setPosition(PARTICLE_DUMPER_IN);

            if (controller2.leftStickY() < 0 && !resurrection.hangTopSwitch.getState()) resurrection.hang.setPower(-1);
            else if (controller2.leftStickY() > 0 && !resurrection.hangBottomSwitch.getState()) resurrection.hang.setPower(1);
            else resurrection.hang.setBreakMode();

            resurrection.collectionLift.DriverControl(controller1);
            resurrection.scoreLift.DriverControl(controller2);

            resurrection.tracker.updateSystem();
            dash.create("2Y: ", controller2.leftStickY());
            dash.create("X: ", resurrection.tracker.getGlobalX());
            dash.create("Y: ", resurrection.tracker.getGlobalY());
            dash.create("H: ", resurrection.tracker.getHeading());
            dash.create("Vert Lift: ", resurrection.scoreLift.getCurrentPosition());
            dash.update();
        }
    }
}
