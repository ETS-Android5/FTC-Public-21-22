package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.firstinspires.ftc.teamcode.core.controller.Namable;

import java.util.function.Function;

public class RunToPositionTracker implements Namable {
    public RunToPositionTracker(String name, Function<Double, Double> powerCurve, int startingTicks, int targetTicks) {
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
    private final Function<Double, Double> powerCurve;
    private int startingTicks;
    private int targetTicks;

    public double getTargetPowerPercentage(int currentTicks) {
        double startingDifference = Math.abs(targetTicks - startingTicks);
        if (startingDifference == 0) return 0;
        double currentDifference = Math.abs(targetTicks - currentTicks);
        if (currentDifference == 0) return 0;
        double progressPercentage = currentTicks > targetTicks
                ? 1 + (currentDifference / startingDifference) : 1 - (currentDifference / startingDifference);
        return powerCurve.apply(progressPercentage) * (startingTicks < targetTicks ? 1 : -1);
    }

    @Override
    public String getName() {
        return name;
    }
}
