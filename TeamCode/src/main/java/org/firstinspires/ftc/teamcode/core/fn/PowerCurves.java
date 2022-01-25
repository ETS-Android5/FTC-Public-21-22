package org.firstinspires.ftc.teamcode.core.fn;

public class PowerCurves {
  public static final TriFunction<Double, Double, Double, Double> CAROUSEL_CURVE =
      (Double currentTicks, Double startingTicks, Double targetTicks) -> {
        if (startingTicks.equals(targetTicks)) return 0.0;
        if (currentTicks.equals(targetTicks)) return 0.0;
        // We haven't reached our target
        if (startingTicks < targetTicks) {
          // Motor should spin forward
          if (currentTicks > targetTicks) return 0.0; // We're past it, stop
          if (currentTicks < startingTicks) return 0.6;
          double percentProgress = (currentTicks - startingTicks) / (targetTicks - startingTicks);
          if (percentProgress > .65) return 1.0;
          if (percentProgress > .1) return 0.8;
          return 0.6;
        } else {
          // Motor should spin backward
          if (currentTicks < targetTicks) return 0.0;
          if (currentTicks > startingTicks) return -0.6;
          double percentProgress = (startingTicks - currentTicks) / (startingTicks - targetTicks);
          if (percentProgress > .65) return -1.0;
          if (percentProgress > .1) return -0.8;
          return -0.6;
        }
      };

  public static final TriFunction<Double, Double, Double, Double> CAROUSEL_CURVE_SLOW =
          (Double currentTicks, Double startingTicks, Double targetTicks) -> {
            if (startingTicks.equals(targetTicks)) return 0.0;
            if (currentTicks.equals(targetTicks)) return 0.0;
            // We haven't reached our target
            if (startingTicks < targetTicks) {
              // Motor should spin forward
              if (currentTicks > targetTicks) return 0.0; // We're past it, stop
              if (currentTicks < startingTicks) return 0.3;
              double percentProgress = (currentTicks - startingTicks) / (targetTicks - startingTicks);
              if (percentProgress > .65) return 1.0;
              if (percentProgress > .1) return 0.4;
              return 0.3;
            } else {
              // Motor should spin backward
              if (currentTicks < targetTicks) return 0.0;
              if (currentTicks > startingTicks) return -0.3;
              double percentProgress = (startingTicks - currentTicks) / (startingTicks - targetTicks);
              if (percentProgress > .65) return -1.0;
              if (percentProgress > .1) return -0.4;
              return -0.3;
            }
          };

  public static TriFunction<Double, Double, Double, Double> generatePowerCurve(
      double maxPower, double rampSlope) {
    return (Double currentTicks, Double startingTicks, Double targetTicks) -> {
      if (startingTicks.equals(targetTicks)) return 0.0;
      if (currentTicks.equals(targetTicks)) return 0.0;
      if (startingTicks < targetTicks) {
        if (currentTicks < startingTicks) return maxPower;
        double percentProgress = (currentTicks - startingTicks) / (targetTicks - startingTicks);
        return Math.max(Math.min(maxPower, rampSlope * (1 - percentProgress)), -maxPower);
      } else {
        // Motor should spin backward
        if (currentTicks > startingTicks) return -maxPower;
        double percentProgress = (startingTicks - currentTicks) / (startingTicks - targetTicks);
        return -Math.max(Math.min(maxPower, rampSlope * (1 - percentProgress)), -maxPower);
      }
    };
  }

  public static TriFunction<Double, Double, Double, Double> generatePowerCurve(
      double maxPower, double rampSlope, double maxAdjustPerTick) {
    return (Double currentTicks, Double startingTicks, Double targetTicks) -> {
      if (startingTicks.equals(targetTicks)) return 0.0;
      if (currentTicks.equals(targetTicks)) return 0.0;
      double positiveMax =
          Math.min(maxPower, maxAdjustPerTick * (Math.abs(currentTicks - targetTicks)));
      double negativeMax = -positiveMax;
      if (startingTicks < targetTicks) {
        double percentProgress = (currentTicks - startingTicks) / (targetTicks - startingTicks);
        return Math.max(Math.min(positiveMax, rampSlope * (1 - percentProgress)), negativeMax);
      } else {
        // Motor should spin backward
        double percentProgress = (startingTicks - currentTicks) / (startingTicks - targetTicks);
        return -Math.max(Math.min(positiveMax, rampSlope * (1 - percentProgress)), negativeMax);
      }
    };
  }
}
