package org.firstinspires.ftc.teamcode.core.fn;

public class PowerCurves {
    public static final TriFunction<Double, Integer, Integer, Double> RUN_TO_POSITION_RAMP = (Double percentProgress, Integer startingTicks, Integer targetTicks) -> {
        if (percentProgress < 0) return 1.0;
        if (percentProgress >= 2) return -1.0;
        double adjustedPercentProgress = percentProgress;
        if (adjustedPercentProgress > 1) {
            adjustedPercentProgress -= ((int) adjustedPercentProgress) - 1;
        }
        double ret = 2.33 * (1 - adjustedPercentProgress);
        if (ret < -1) {
            return -1.0;
        } else if (ret > 1) {
            return 1.0;
        } else {
            return ret;
        }
    };

    public static final TriFunction<Double, Integer, Integer, Double> CAROUSEL_CURVE = (Double percentProgress, Integer startingTicks, Integer targetTicks) -> {
        if (startingTicks < targetTicks && percentProgress < 0) return 1.0;
        if (startingTicks < targetTicks && percentProgress >= 1) return 0.0;
        if (startingTicks > targetTicks && percentProgress < 0) return 0.0;
        if (startingTicks > targetTicks && percentProgress >= 1) return -1.0;
        double adjustedPercentProgress = percentProgress;
        if (adjustedPercentProgress > 1) {
            adjustedPercentProgress -= ((int) adjustedPercentProgress) - 1;
        }
        double ret = 20 * (1 - adjustedPercentProgress);
        if (ret < -1) {
            return -1.0;
        } else if (ret > 1) {
            return 1.0;
        } else {
            return ret;
        }
    };
}
