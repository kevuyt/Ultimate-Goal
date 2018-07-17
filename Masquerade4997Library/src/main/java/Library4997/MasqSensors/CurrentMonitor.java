package Library4997.MasqSensors;

/**
 * Created by Archishmaan Peyyety on 7/17/18.
 * Project: MasqLib
 */

import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import java.lang.reflect.Method;

import Library4997.MasqMotors.MasqMotor;

public class CurrentMonitor {
    LynxModule hubOne;
    LynxModule hubTwo;

    public CurrentMonitor(LinearOpMode opMode, String hubOneName, String hubTwoName) {
        hubOne = opMode.hardwareMap.get(LynxModule.class, hubOneName);
        hubTwo = opMode.hardwareMap.get(LynxModule.class, hubTwoName);
    }

    public CurrentMonitor(LinearOpMode opMode) {
        this(opMode, "Expansion Hub 1", "Expansion Hub 2");
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
        if (motorPort == 0) channel = LynxGetADCCommand.Channel.MOTOR0_CURRENT;
        else if (motorPort == 1) channel = LynxGetADCCommand.Channel.MOTOR1_CURRENT;
        else if (motorPort == 2) channel = LynxGetADCCommand.Channel.MOTOR2_CURRENT;
        else if (motorPort == 3) channel = LynxGetADCCommand.Channel.MOTOR3_CURRENT;
        else return 0;
        return getADCValue(hub, channel);
    }

    public double getADCValue(int whichHub, LynxGetADCCommand.Channel channel) {
        LynxModule hub;
        if (whichHub == 1)
            return getADCValue(hubOne, channel);
        else if (whichHub == 2)
            return getADCValue(hubTwo, channel);
        else
            return 0;
    }

    private double getADCValue(LynxModule hub, LynxGetADCCommand.Channel channel) {
        LynxGetADCCommand command = new LynxGetADCCommand(hub, LynxGetADCCommand.Channel.BATTERY_CURRENT, LynxGetADCCommand.Mode.ENGINEERING);
        try {
            LynxGetADCResponse response = command.sendReceive();
            return response.getValue();
        } catch (Exception e) {
            RobotLog.ee("ADCMonitor", "failed to communicate with lynx module");
        }
        return 0;
    }

}




