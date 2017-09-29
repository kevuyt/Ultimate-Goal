package Library4997.MasqExternal;

import com.qualcomm.robotcore.hardware.HardwareMap;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 9/28/17.
 */

public class MasqOpModeInternal {
    private MasqLinearOpMode masqLinearOpMode;
    private HardwareMap hardwareMap;
    private static MasqOpModeInternal instance;
    public MasqOpModeInternal (MasqLinearOpMode linearOpMode) {
        masqLinearOpMode = linearOpMode;
    }
    public static MasqOpModeInternal getOpModeIsActive(MasqLinearOpMode linearOpMode){
        if (instance==null)
            instance = new MasqOpModeInternal(linearOpMode);
        return instance;
    }
    public boolean opModeIsActive() {return masqLinearOpMode.opModeIsActive();}
    public HardwareMap getHardwareMap (){
        return masqLinearOpMode.hardwareMap;
    }
}
