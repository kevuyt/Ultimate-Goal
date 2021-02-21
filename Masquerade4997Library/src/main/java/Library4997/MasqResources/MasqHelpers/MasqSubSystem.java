package Library4997.MasqResources.MasqHelpers;

import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Archish on 2/12/18.
 */

public interface MasqSubSystem {
    void driverControl(MasqController controller) throws InterruptedException;
    String getName();
    MasqHardware[] getComponents();
}
