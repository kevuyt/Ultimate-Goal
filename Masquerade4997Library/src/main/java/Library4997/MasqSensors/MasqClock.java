package Library4997.MasqSensors;

import android.annotation.SuppressLint;

import java.util.Locale;

import Library4997.MasqResources.MasqHardware;

import static java.lang.System.nanoTime;
import static java.util.Locale.US;


public class MasqClock implements MasqHardware {

    private long startTime;


    public MasqClock() {this.reset();}
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
        return nanoseconds() <= (long) (time * resolution.multiplier);
    }
    public boolean hasNotPassed(double time) {
        return hasNotPassed(time, Resolution.SECONDS);
    }

    public String getName() {return "CLOCK";}
    public String[] getDash() {
        return new String[]{
                String.format(US, "Seconds passed: %.5f", seconds())
        };
    }

}