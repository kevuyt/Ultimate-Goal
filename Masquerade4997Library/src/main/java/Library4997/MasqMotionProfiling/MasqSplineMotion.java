package Library4997.MasqMotionProfiling;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Archish on 5/13/18.
 */

public class MasqSplineMotion {
    private List<MasqPoint> points = new ArrayList<>();
    public void setPoints(MasqPoint... points) {
        for (MasqPoint point: points) this.points.add(point);
    }
    public List<MasqPoint> getPoints () {
        return points;
    }
}
