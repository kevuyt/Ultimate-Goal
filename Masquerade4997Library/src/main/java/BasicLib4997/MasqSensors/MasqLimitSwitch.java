package BasicLib4997.MasqSensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import BasicLib4997.MasqHardware;


/**
 * This is a basic limit switch.
 * Assumes that a pull-up resistor is used, like is necessary to use VEX limit switches.
 */

public class MasqLimitSwitch implements MasqHardware {

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
    public boolean onRealese() {
        return true;
    }
    // Possibly add onContact() and onRelease() methods for more precise control


    public String getName() {return name;}
    public String getDash() {return Boolean.toString(getState());}

}