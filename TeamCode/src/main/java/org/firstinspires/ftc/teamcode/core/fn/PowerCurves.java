package org.firstinspires.ftc.teamcode.core.fn;

public class PowerCurves {
    public static final TriFunction<Integer, Integer, Integer, Double> RUN_TO_POSITION_RAMP =
            (Integer currentTicks, Integer startingTicks, Integer targetTicks) -> {
        if (startingTicks.equals(targetTicks)) return 0.0;
        if (currentTicks.equals(targetTicks)) return 0.0;
        // We haven't reached our target
        if (startingTicks < targetTicks) {
            // Motor should spin forward
            if (currentTicks < startingTicks) return 1.0;
            double percentProgress = ((double) (currentTicks - startingTicks)) / ((double) (targetTicks - startingTicks));
            return 2.33 * (1 - percentProgress);
        } else {
            // Motor should spin backward
            if (currentTicks > startingTicks) return -1.0;
            double percentProgress = ((double) (startingTicks - currentTicks)) / ((double) (startingTicks - targetTicks));
            return -2.33 * (1 - percentProgress);
        }
    };

    public static final TriFunction<Integer, Integer, Integer, Double> RUN_TO_POSITION_HALF_POWER =
            (Integer currentTicks, Integer startingTicks, Integer targetTicks) -> {
                if (startingTicks.equals(targetTicks)) return 0.0;
                if (currentTicks.equals(targetTicks)) return 0.0;
                // We haven't reached our target
                if (startingTicks < targetTicks) {
                    // Motor should spin forward
                    if (currentTicks < startingTicks) return 0.5;
                    double percentProgress = ((double) (currentTicks - startingTicks)) / ((double) (targetTicks - startingTicks));
                    return 2.33 * (1 - percentProgress);
                } else {
                    // Motor should spin backward
                    if (currentTicks > startingTicks) return -0.5;
                    double percentProgress = ((double) (startingTicks - currentTicks)) / ((double) (startingTicks - targetTicks));
                    return -2.33 * (1 - percentProgress);
                }
    };

    public static final TriFunction<Integer, Integer, Integer, Double> RUN_TO_POSITION_QUARTER_POWER =
            (Integer currentTicks, Integer startingTicks, Integer targetTicks) -> {
                if (startingTicks.equals(targetTicks)) return 0.0;
                if (currentTicks.equals(targetTicks)) return 0.0;
                // We haven't reached our target
                if (startingTicks < targetTicks) {
                    // Motor should spin forward
                    if (currentTicks < startingTicks) return 0.25;
                    double percentProgress = ((double) (currentTicks - startingTicks)) / ((double) (targetTicks - startingTicks));
                    return 2.33 * (1 - percentProgress);
                } else {
                    // Motor should spin backward
                    if (currentTicks > startingTicks) return -0.25;
                    double percentProgress = ((double) (startingTicks - currentTicks)) / ((double) (startingTicks - targetTicks));
                    return -2.33 * (1 - percentProgress);
                }
    };

    public static final TriFunction<Integer, Integer, Integer, Double> CAROUSEL_CURVE =
            (Integer currentTicks, Integer startingTicks, Integer targetTicks) -> {
        if (startingTicks.equals(targetTicks)) return 0.0;
        if (currentTicks.equals(targetTicks)) return 0.0;
        // We haven't reached our target
        if (startingTicks < targetTicks) {
            // Motor should spin forward
            if (currentTicks > targetTicks) return 0.0; // We're past it, stop
            if (currentTicks < startingTicks) return 0.6;
            double percentProgress = ((double) (currentTicks - startingTicks)) / ((double) (targetTicks - startingTicks));
            if (percentProgress > .65) return 1.0;
            if (percentProgress > .1) return 0.8;
            return 0.6;
        } else {
            // Motor should spin backward
            if (currentTicks < targetTicks) return 0.0;
            if (currentTicks > startingTicks) return -0.6;
            double percentProgress = ((double) (startingTicks - currentTicks)) / ((double) (startingTicks - targetTicks));
            if (percentProgress > .65) return -1.0;
            if (percentProgress > .1) return -0.8;
            return -0.6;
        }
    };
}
