package org.firstinspires.ftc.teamcode.hardware.detection.distance;

public class VL53L1X_Result {
  private byte status;
  private short distance;
  private short ambient;
  private short sigPerSPAD;
  private short numSPADS;

  public VL53L1X_Result(
      byte status, short distance, short ambient, short sigPerSPAD, short numSPADS) {
    this.status = status;
    this.distance = distance;
    this.ambient = ambient;
    this.sigPerSPAD = sigPerSPAD;
    this.numSPADS = numSPADS;
  }

  @Override
  public String toString() {
    return "/*******VL53L1X_Result********\n"
        + "STATUS\t"
        + status
        + "\n"
        + "DISTANCE\t"
        + distance
        + "\n"
        + "AMBIENT\t"
        + ambient
        + "\n"
        + "SIGPERSPAD\t"
        + sigPerSPAD
        + "\n"
        + "NUMSPADS\t"
        + numSPADS
        + "\n"
        + "********VL53L1X_Result*******/\n";
  }

  public byte getStatus() {
    return status;
  }

  public void setStatus(byte status) {
    this.status = status;
  }

  public short getDistance() {
    return distance;
  }

  public void setDistance(short distance) {
    this.distance = distance;
  }

  public short getAmbient() {
    return ambient;
  }

  public void setAmbient(short ambient) {
    this.ambient = ambient;
  }

  public short getSigPerSPAD() {
    return sigPerSPAD;
  }

  public void setSigPerSPAD(short sigPerSPAD) {
    this.sigPerSPAD = sigPerSPAD;
  }

  public short getNumSPADS() {
    return numSPADS;
  }

  public void setNumSPADS(short numSPADS) {
    this.numSPADS = numSPADS;
  }
}
