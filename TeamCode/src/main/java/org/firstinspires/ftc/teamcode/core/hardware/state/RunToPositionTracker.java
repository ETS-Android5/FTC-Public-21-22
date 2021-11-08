package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.firstinspires.ftc.teamcode.core.controller.Namable;

public class RunToPositionTracker implements Namable {
    public RunToPositionTracker(String name, int startingTicks, int targetTicks) {
        this.name = name;
        this.startingTicks = startingTicks;
        this.targetTicks = targetTicks;
    }

    public void mutateTo(int startingTicks, int targetTicks) {
        this.startingTicks = startingTicks;
        this.targetTicks = targetTicks;
    }

    private final String name;
    private int startingTicks;
    private int targetTicks;

    public int getStartingTicks() {
        return startingTicks;
    }

    public int getTargetTicks() {
        return targetTicks;
    }

    public double getTargetPowerPercentage(int currentTicks) {
        double startingDifference = Math.abs(targetTicks - startingTicks);
        double currentDifference = Math.abs(targetTicks - currentTicks);
        double progressPercentage = (startingDifference - currentDifference) / startingDifference;
        progressPercentage = progressPercentage < 0 ? 0 : progressPercentage > 1 ? 1 : progressPercentage;
        double power = Math.abs(1 - progressPercentage) > .02 ? 1 - Math.pow(progressPercentage, 6) : 0;
        return power * (currentTicks < targetTicks ? 1 : -1);
    }

    @Override
    public String getName() {
        return name;
    }
}
