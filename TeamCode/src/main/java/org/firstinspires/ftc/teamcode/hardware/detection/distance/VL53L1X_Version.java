package org.firstinspires.ftc.teamcode.hardware.detection.distance;

public class VL53L1X_Version {
    public VL53L1X_Version(short major, short minor, short build, long revision) {
        this.major = major;
        this.minor = minor;
        this.build = build;
        this.revision = revision;
    }

    public short getMajor() {
        return major;
    }

    public void setMajor(short major) {
        this.major = major;
    }

    public short getMinor() {
        return minor;
    }

    public void setMinor(short minor) {
        this.minor = minor;
    }

    public short getBuild() {
        return build;
    }

    public void setBuild(short build) {
        this.build = build;
    }

    public long getRevision() {
        return revision;
    }

    public void setRevision(long revision) {
        this.revision = revision;
    }

    private short major;
    private short minor;
    private short build;
    private long revision;
}
