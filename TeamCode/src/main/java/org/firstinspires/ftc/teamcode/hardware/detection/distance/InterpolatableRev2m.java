package org.firstinspires.ftc.teamcode.hardware.detection.distance;

import com.google.common.collect.EvictingQueue;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.hardware.state.DataPoint;
import org.firstinspires.ftc.teamcode.core.hardware.state.Interpolatable;

@SuppressWarnings("UnstableApiUsage")
public class InterpolatableRev2m implements Interpolatable {
  private final String name;

  @Hardware(name = "")
  public Rev2mDistanceSensor distanceSensor;

  private EvictingQueue<DataPoint> dataPoints;
  private long sampleRateNs;
  private int polynomialDegree;
  private long lastSampleTime;
  private boolean isSampling = false;

  public InterpolatableRev2m(
      int analysisSize, long sampleRateNs, int polynomialDegree, String name) {
    this.dataPoints = EvictingQueue.create(analysisSize);
    this.sampleRateNs = sampleRateNs;
    this.polynomialDegree = polynomialDegree;
    this.lastSampleTime = 0;
    this.name = name;
  }

  @Override
  public EvictingQueue<DataPoint> getDataPoints() {
    return dataPoints;
  }

  @Override
  public int getAnalysisSize() {
    return dataPoints.size() + dataPoints.remainingCapacity();
  }

  @Override
  public void setAnalysisSize(int size) {
    EvictingQueue<DataPoint> nextQueue = EvictingQueue.create(size);
    while (!dataPoints.isEmpty()) {
      DataPoint element = dataPoints.poll();
      if (element != null) {
        nextQueue.add(element);
      }
    }
    dataPoints = nextQueue;
  }

  @Override
  public long getSampleRateNs() {
    return sampleRateNs;
  }

  @Override
  public void setSampleRateNs(long sampleRateNs) {
    this.sampleRateNs = sampleRateNs;
  }

  @Override
  public int getPolynomialDegree() {
    return polynomialDegree;
  }

  @Override
  public void setPolynomialDegree(int polynomialDegree) {
    this.polynomialDegree = polynomialDegree;
  }

  @Override
  public long timeToNextSample() {
    return lastSampleTime == 0 ? 0 : (lastSampleTime + sampleRateNs) - System.nanoTime();
  }

  @Override
  public void sample() {
    // Getting new data point
    double distance = distanceSensor.getDistance(DistanceUnit.MM);
    distance = distance > 2000 ? 2000 : distance;
    distance = distance < 0 ? 0 : distance;
    lastSampleTime = System.nanoTime();
    dataPoints.add(new DataPoint(distance, lastSampleTime));
  }

  @Override
  public void initialize() {}

  @Override
  public boolean isSampling() {
    return this.isSampling;
  }

  @Override
  public void startSampling() {
    dataPoints.clear();
    isSampling = true;
  }

  @Override
  public void stopSampling() {
    dataPoints.clear();
    isSampling = false;
  }

  @Override
  public String getName() {
    return name;
  }
}
