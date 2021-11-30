package org.firstinspires.ftc.teamcode.core.hardware.state;

public class DataPoint {
    private final double data;
    private final long timestamp;

    public DataPoint(double data, long timestamp) {
        this.data = data;
        this.timestamp = timestamp;
    }

    public double getData() {
        return data;
    }

    public long getTimestamp() {
        return timestamp;
    }
}
