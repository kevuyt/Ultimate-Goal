package MasqLibrary.MasqResources;

import androidx.annotation.NonNull;

import static java.lang.System.nanoTime;
import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqClock {
    private long startTime;
    String name;

    public MasqClock(String name) {
        reset();
        this.name = name;
    }
    public MasqClock() {this("Clock");}

    public void reset() {startTime = nanoTime();}

    public long nanoseconds() {return nanoTime() - startTime;}
    public double milliseconds() {return nanoseconds() * 1E-6;}
    public double seconds() {return nanoseconds() * 1E-9;}

    public enum Resolution {
        NANOSECONDS (1),
        MILLISECONDS (1E6),
        SECONDS (1E9);
        private final double multiplier;
        Resolution (double multiplier) {this.multiplier = multiplier;}
    }

    public boolean hasNotPassed(double time, Resolution resolution) {
        return nanoseconds() < (long) (time * resolution.multiplier);
    }
    public boolean hasNotPassed(double time) {return hasNotPassed(time, Resolution.SECONDS);}

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "%s:\nSeconds Passed: %f", name, seconds());
    }
}