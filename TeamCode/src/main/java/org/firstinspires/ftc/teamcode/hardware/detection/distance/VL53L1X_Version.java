package org.firstinspires.ftc.teamcode.hardware.detection.distance;

public class VL53L1X_Version {
  public VL53L1X_Version(byte major, byte minor, byte build, byte revision) {
    this.major = major;
    this.minor = minor;
    this.build = build;
    this.revision = revision;
  }

  public byte getMajor() {
    return major;
  }

  public void setMajor(byte major) {
    this.major = major;
  }

  public byte getMinor() {
    return minor;
  }

  public void setMinor(byte minor) {
    this.minor = minor;
  }

  public byte getBuild() {
    return build;
  }

  public void setBuild(byte build) {
    this.build = build;
  }

  public long getRevision() {
    return revision;
  }

  public void setRevision(byte revision) {
    this.revision = revision;
  }

  private byte major;
  private byte minor;
  private byte build;
  private byte revision;
}
