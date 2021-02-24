package Library4997.MasqResources;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Archish on 2/12/18.
 */

public interface MasqSubSystem {
    void driverControl(Gamepad controller) ;
    String getName();
    MasqHardware[] getComponents();
}
