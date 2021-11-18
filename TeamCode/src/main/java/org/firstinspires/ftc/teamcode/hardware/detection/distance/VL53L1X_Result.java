package org.firstinspires.ftc.teamcode.hardware.detection.distance;

public class VL53L1X_Result {
  public VL53L1X_Result(short status, int distance, int ambient, int sigPerSPAD, int numSPADS) {
    this.status = status;
    this.distance = distance;
    this.ambient = ambient;
    this.sigPerSPAD = sigPerSPAD;
    this.numSPADS = numSPADS;
  }

  public short getStatus() {
    return status;
  }

  public void setStatus(short status) {
    this.status = status;
  }

  public int getDistance() {
    return distance;
  }

  public void setDistance(int distance) {
    this.distance = distance;
  }

  public int getAmbient() {
    return ambient;
  }

  public void setAmbient(int ambient) {
    this.ambient = ambient;
  }

  public int getSigPerSPAD() {
    return sigPerSPAD;
  }

  public void setSigPerSPAD(int sigPerSPAD) {
    this.sigPerSPAD = sigPerSPAD;
  }

  public int getNumSPADS() {
    return numSPADS;
  }

  public void setNumSPADS(int numSPADS) {
    this.numSPADS = numSPADS;
  }

  private short status;
  private int distance;
  private int ambient;
  private int sigPerSPAD;
  private int numSPADS;
}
