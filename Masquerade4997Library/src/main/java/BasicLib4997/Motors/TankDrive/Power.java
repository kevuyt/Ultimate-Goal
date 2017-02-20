package BasicLib4997.Motors.TankDrive;

/**
 * Created by Archish on 12/25/16.
 */

public enum Power {
    Optimal (+0.5),
    High (+0.7),
    Low (+0.3);
    public final double value;
    Power (double value) {this.value = value;}
}
