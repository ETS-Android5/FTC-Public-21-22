package org.firstinspires.ftc.teamcode.core.fn;

import java.util.function.Function;

public class PowerCurves {
    public static final Function<Double, Double> RUN_TO_POSITION_RAMP = (Double percentProgress) -> {
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

    public static final Function<Double, Double> CAROUSEL_CURVE = (Double percentProgress) -> {
        if (percentProgress < 0) return 1.0;
        if (percentProgress >= 2) return -1.0;
        double direction = 1.0;
        double adjustedPercentProgress = percentProgress;
        if (adjustedPercentProgress > 1) {
            adjustedPercentProgress -= (int) adjustedPercentProgress;
            direction = -1;
        }

        if (adjustedPercentProgress <= .25) {
            return Math.max(25.6 * Math.pow(adjustedPercentProgress, 3), .1) * direction;
        } else if (adjustedPercentProgress > .25 && adjustedPercentProgress < .75) {
            return .4 * direction;
        } else {
            return direction;
        }
    };
}
