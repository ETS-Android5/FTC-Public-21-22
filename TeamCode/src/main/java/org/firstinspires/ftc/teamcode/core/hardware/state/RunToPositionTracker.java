package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.firstinspires.ftc.teamcode.core.controller.Namable;
import org.firstinspires.ftc.teamcode.core.fn.TriFunction;

public class RunToPositionTracker implements Namable {
    public RunToPositionTracker(String name, TriFunction<Double, Integer, Integer, Double> powerCurve, int startingTicks, int targetTicks) {
        this.name = name;
        this.powerCurve = powerCurve;
        this.startingTicks = startingTicks;
        this.targetTicks = targetTicks;
    }

    public void mutateTo(int startingTicks, int targetTicks) {
        this.startingTicks = startingTicks;
        this.targetTicks = targetTicks;
    }

    private final String name;
    private final TriFunction<Double, Integer, Integer, Double> powerCurve;
    private int startingTicks;
    private int targetTicks;

    public double getTargetPowerPercentage(int currentTicks) {
        return powerCurve.apply(getProgressPercentage(currentTicks), startingTicks, targetTicks);
    }

    private double getProgressPercentage(int currentTicks) {
        double startingDifference = Math.abs(targetTicks - startingTicks);
        if (startingDifference == 0) return 0;
        double currentDifference = Math.abs(targetTicks - currentTicks);
        if (currentDifference == 0) return 1;
        if (currentTicks > targetTicks) return 1 + (currentDifference / startingDifference);
        return 1 - (currentDifference / startingDifference);
    }

    @Override
    public String getName() {
        return name;
    }
}
