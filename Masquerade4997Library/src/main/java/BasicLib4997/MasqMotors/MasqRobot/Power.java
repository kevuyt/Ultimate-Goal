package BasicLib4997.MasqMotors.MasqRobot;

/**
 * Enums For Levels of Powers
 */

public enum Power {
    Optimal (+0.5),
    High (+0.7),
    Low (+0.3);
    public final double value;
    Power (double value) {this.value = value;}
}
