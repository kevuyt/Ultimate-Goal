package SubSystems4997;

import Library4997.MasqUtilities.MasqHelpers.MasqHardware;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Archish on 2/12/18.
 */

public interface MasqSubSystem {
    void DriverControl(MasqController controller);
    String getName();
    MasqHardware[] getComponents();
}
