package Library4997.MasqSensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import Library4997.MasqHardware;
import Library4997.MasqSensor;


/**
 * This is a basic limit switch.
 * Assumes that a pull-up resistor is used, like is necessary to use VEX limit switches.
 */

public class MasqLimitSwitch implements MasqHardware, MasqSensor {

    private final DigitalChannel limitSwitch;
    private final String name;

    private boolean signalValue;
    private boolean logicalState;


    public MasqLimitSwitch(String name) {
        this.name = name;
        limitSwitch = FtcOpModeRegister.opModeManager.getHardwareMap().digitalChannel.get(name);
        limitSwitch.setMode(DigitalChannelController.Mode.INPUT);
    }
    public void updateState() {
        signalValue = limitSwitch.getState();
        logicalState = !signalValue;
    }
    public boolean getSignalValue() {
        updateState();
        return signalValue;
    }

    public boolean getState() {
        updateState();
        return logicalState;
    }

    public boolean isPressed() {return getState();}
    public boolean pr() {
        boolean pressed = false, released = false;
        while (isPressed()) {
            released = false;
            pressed = true;
        }
        while (!isPressed()) released = true;
        return pressed && released;
    }
    public boolean isReleased () {
        boolean released = false;
        while (isPressed()) released = false;
        while (!isPressed()) released = true;
        return released;
    }
    public String getName() {return name;}
    public String[] getDash() {return new String[] {
            "Is Pressed" + Boolean.toString(isPressed()),
            "Pressed and Released" + Boolean.toString(pr()),
            "Released" + Boolean.toString(isReleased())
        };
    }
    public boolean stop() {
        return !isPressed();
    }
}