package Library4997.MasqSensors;

import com.qualcomm.hardware.lynx.*;
import com.qualcomm.hardware.lynx.commands.core.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.lang.reflect.Method;

import Library4997.MasqMotors.MasqMotor;

import static com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand.Channel.*;

/**
 * Created by Archishmaan Peyyety on 7/17/18.
 * Project: MasqLib
 */

public class MasqCurrentMonitor {
    LynxModule hubOne;
    LynxModule hubTwo;

    public MasqCurrentMonitor(HardwareMap hardwareMap, String hubOneName, String hubTwoName) {
        hubOne = hardwareMap.get(LynxModule.class, hubOneName);
        hubTwo = hardwareMap.get(LynxModule.class, hubTwoName);
    }

    public MasqCurrentMonitor(HardwareMap hardwareMap, int hub1Number, int hub2Number) {
        this(hardwareMap, "Expansion Hub " + hub1Number, "Expansion Hub " + hub2Number);
    }

    public double getMotorCurrent(MasqMotor motor) {
        LynxDcMotorController controller = (LynxDcMotorController) motor.getController();
        LynxModule hub = null;
        try {
            Method modMethod = LynxController.class.getDeclaredMethod("getModule");
            modMethod.setAccessible(true);

            hub = (LynxModule) modMethod.invoke(controller);

        } catch (Exception e) {
            RobotLog.ee("ADCMonitor", "failed to communicate with lynx module");
        }
        if (hub == null) return 0;
        LynxGetADCCommand.Channel channel;
        int motorPort = motor.getPortNumber();
        if (motorPort == 0) channel = MOTOR0_CURRENT;
        else if (motorPort == 1) channel = MOTOR1_CURRENT;
        else if (motorPort == 2) channel = MOTOR2_CURRENT;
        else if (motorPort == 3) channel = MOTOR3_CURRENT;
        else return 0;
        return getADCValue(hub, channel);
    }

    public double getADCValue(int whichHub, LynxGetADCCommand.Channel channel) {
        if (whichHub == 1) return getADCValue(hubOne, channel);
        else if (whichHub == 2) return getADCValue(hubTwo, channel);
        else return 0;
    }

    private double getADCValue(LynxModule hub, LynxGetADCCommand.Channel channel) {
        LynxGetADCCommand command = new LynxGetADCCommand(hub, channel, LynxGetADCCommand.Mode.ENGINEERING);
        try {
            LynxGetADCResponse response = command.sendReceive();
            return response.getValue();
        } catch (Exception e) {
            RobotLog.ee("ADCMonitor", "failed to communicate with lynx module");
        }
        return 0;
    }

}




