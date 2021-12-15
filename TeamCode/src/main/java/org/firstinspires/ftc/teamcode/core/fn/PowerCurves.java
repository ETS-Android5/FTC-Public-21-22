package org.firstinspires.ftc.teamcode.core.fn;

public class PowerCurves {
  public static final TriFunction<Integer, Integer, Integer, Double> CAROUSEL_CURVE =
      (Integer currentTicks, Integer startingTicks, Integer targetTicks) -> {
        if (startingTicks.equals(targetTicks)) return 0.0;
        if (currentTicks.equals(targetTicks)) return 0.0;
        // We haven't reached our target
        if (startingTicks < targetTicks) {
          // Motor should spin forward
          if (currentTicks > targetTicks) return 0.0; // We're past it, stop
          if (currentTicks < startingTicks) return 0.6;
          double percentProgress =
              ((double) (currentTicks - startingTicks)) / ((double) (targetTicks - startingTicks));
          if (percentProgress > .65) return 1.0;
          if (percentProgress > .1) return 0.8;
          return 0.6;
        } else {
          // Motor should spin backward
          if (currentTicks < targetTicks) return 0.0;
          if (currentTicks > startingTicks) return -0.6;
          double percentProgress =
              ((double) (startingTicks - currentTicks)) / ((double) (startingTicks - targetTicks));
          if (percentProgress > .65) return -1.0;
          if (percentProgress > .1) return -0.8;
          return -0.6;
        }
      };

  public static TriFunction<Integer, Integer, Integer, Double> generateDynamicPowerCurve(
      double maxPower, double rampSlope, int ticksToRampTo) {
    TriFunction<Integer, Integer, Integer, Double> basePowerCurve =
        generatePowerCurve(maxPower, rampSlope);
    return (Integer currentTicks, Integer startingTicks, Integer targetTicks) ->
        Math.min(Math.abs(currentTicks - targetTicks) / ticksToRampTo, 1)
            * basePowerCurve.apply(currentTicks, startingTicks, targetTicks);
  }

  public static TriFunction<Integer, Integer, Integer, Double> generatePowerCurve(
      double maxPower, double rampSlope) {
    return (Integer currentTicks, Integer startingTicks, Integer targetTicks) -> {
      if (startingTicks.equals(targetTicks)) return 0.0;
      if (currentTicks.equals(targetTicks)) return 0.0;
      if (startingTicks < targetTicks) {
        if (currentTicks < startingTicks) return maxPower;
        double percentProgress =
            ((double) (currentTicks - startingTicks)) / ((double) (targetTicks - startingTicks));
        return Math.max(Math.min(maxPower, rampSlope * (1 - percentProgress)), -maxPower);
      } else {
        // Motor should spin backward
        if (currentTicks > startingTicks) return -maxPower;
        double percentProgress =
            ((double) (startingTicks - currentTicks)) / ((double) (startingTicks - targetTicks));
        return -Math.max(Math.min(maxPower, rampSlope * (1 - percentProgress)), -maxPower);
      }
    };
  }
}
