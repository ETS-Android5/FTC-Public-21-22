package org.firstinspires.ftc.teamcode.core.hardware.state;

public class MotorPositionData {
    private long previousTime;
    private double previousTicks;
    private long currentTime;
    private double currentTicks;

    public long getPreviousTime() {
        return previousTime;
    }

    public double getPreviousTicks() {
        return previousTicks;
    }

    public long getCurrentTime() {
        return currentTime;
    }

    public double getCurrentTicks() {
        return currentTicks;
    }

    public MotorPositionData(long previousTick, int previousCount, long currentTick, int currentCount) {
        this.previousTime = previousTick;
        this.previousTicks = previousCount;
        this.currentTime = currentTick;
        this.currentTicks = currentCount;
    }

    public MotorPositionData(long currentTime, int currentTicks) {
        this.currentTime = currentTime;
        this.currentTicks = currentTicks;
    }

    public void addDataPoint(long timestamp, int ticks) {
        previousTime = currentTime;
        previousTicks = currentTicks;
        currentTime = timestamp;
        currentTicks = ticks;
    }

    public double velocity() {
        return (currentTicks - previousTicks) / ((currentTime - previousTime) / 1_000_000_000.0);
    }

}
