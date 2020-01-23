package Library4997.MasqControlSystems.MasqPurePursuit;

import Library4997.MasqResources.MasqMath.MasqPoint;

/**
 * Created by Archishmaan Peyyety on 2020-01-23.
 * Project: MasqLib
 */
public class MasqWayPointV2 {
    private double x, y, h, targetRadius, modeSwitchRadius, pointSwitchRadius,
            minVelocity, maxVelocity, timeout, lookAhead, angularCorrectionSpeed;

    public MasqWayPointV2(double x, double y, double h, double targetRadius, double modeSwitchRadius,
                          double pointSwitchRadius, double minVelocity, double maxVelocity, double timeout,
                          double lookAhead, double angularCorrectionSpeed) {
        this.x = x;
        this.y = y;
        this.h = h;
        this.targetRadius = targetRadius;
        this.modeSwitchRadius = modeSwitchRadius;
        this.pointSwitchRadius = pointSwitchRadius;
        this.minVelocity = minVelocity;
        this.maxVelocity = maxVelocity;
        this.timeout = timeout;
        this.lookAhead = lookAhead;
        this.angularCorrectionSpeed = angularCorrectionSpeed;
    }
    public MasqWayPointV2(MasqPoint p, double targetRadius, double modeSwitchRadius, double pointSwitchRadius,
                          double minVelocity, double maxVelocity, double timeout, double lookAhead, double angularCorrectionSpeed) {
        this.x = p.getX();
        this.y = p.getY();
        this.h = p.getH();
        this.targetRadius = targetRadius;
        this.modeSwitchRadius = modeSwitchRadius;
        this.pointSwitchRadius = pointSwitchRadius;
        this.minVelocity = minVelocity;
        this.maxVelocity = maxVelocity;
        this.timeout = timeout;
        this.lookAhead = lookAhead;
        this.angularCorrectionSpeed = angularCorrectionSpeed;
    }
}
