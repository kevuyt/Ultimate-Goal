package Library4997.MasqExternal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Library4997.MasqWrappers.DashBoard;

/**
 * Created by Archish on 9/28/17.
 */

public class MasqOpModeInternal {
    private HardwareMap hardwareMap;
    public MasqOpModeInternal(HardwareMap hardwareMap, boolean opModeIsActive){
        this.hardwareMap  = hardwareMap;
        instance = this;
    }
    public static MasqOpModeInternal getDash(){return instance;}
    public static MasqOpModeInternal instance;

}
