package org.firstinspires.ftc.teamcode.Robots.Reserection.ResurrectionSubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.atomic.AtomicInteger;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqRobot;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Archishmaan Peyyety on 4/17/19.
 * Project: MasqLib
 */
public class MasqCollectorDumper extends MasqMotor implements MasqSubSystem, Runnable {

    public enum Positions {
        TRANSFER(0),
        HORIZONTAL(540),
        DOWN(1130);
        private int d;
        Positions(int d1){d = d1;}
        public int getPosition() {return d;}
    }

    private AtomicInteger autoPosition = new AtomicInteger(0);
    private MasqPIDController collectionDumpHold = new MasqPIDController(0.02, 0, 0);
    private double targetPosition = 0;

    public MasqCollectorDumper(String name, HardwareMap hardwareMap) {
        super(name, MasqMotorModel.NEVEREST60, hardwareMap);
        resetEncoder();
    }

    @Override
    public void setPower(double power) {
        autoPosition.set((int) getCurrentPosition());
        super.setPower(power);
    }

    @Override
    public void DriverControl(MasqController controller) {
        if (controller.dPadUp()) targetPosition = Positions.TRANSFER.getPosition();
        else if (controller.dPadDown()) targetPosition = Positions.DOWN.getPosition();
        else if (controller.dPadRight()) targetPosition = Positions.HORIZONTAL.getPosition();
        else setPower(collectionDumpHold.getOutput(getCurrentPosition(), targetPosition));
    }

    public void startPositionThread() {
        Thread t = new Thread(this);
        t.start();
    }

    public void setAutoPosition(Positions p) {
        autoPosition.set(p.getPosition());
    }

    public double getTargetPosition() {
        return autoPosition.get();
    }


    @Override
    public MasqHardware[] getComponents() {
        return new MasqHardware[0];
    }

    @Override
    public void run() {
        while (MasqRobot.opModeIsActive())
        super.setPower(collectionDumpHold.getOutput(getCurrentPosition(), autoPosition.get()));
    }
}
