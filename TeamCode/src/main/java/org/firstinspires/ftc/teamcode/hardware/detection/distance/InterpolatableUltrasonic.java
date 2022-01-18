package org.firstinspires.ftc.teamcode.hardware.detection.distance;

import com.google.common.collect.EvictingQueue;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.PollingSubscription;
import org.firstinspires.ftc.teamcode.core.hardware.state.DataPoint;
import org.firstinspires.ftc.teamcode.core.hardware.state.Interpolatable;

@SuppressWarnings("UnstableApiUsage")
public class InterpolatableUltrasonic implements Interpolatable {
  private static final double VOLTS_TO_INCHES_CONVERSION = 1.0 / 0.0049;
  private final String name;

  @Hardware(name = "")
  public AnalogInput distanceSensor;

  private EvictingQueue<DataPoint> dataPoints;
  private int polynomialDegree;

  private boolean isSampling;

  private PollingSubscription pollingSubscription;

  public InterpolatableUltrasonic(int analysisSize, int polynomialDegree, String name) {
    this.dataPoints = EvictingQueue.create(analysisSize);
    this.polynomialDegree = polynomialDegree;
    this.name = name;
    this.isSampling = false;
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
    return 0;
  }

  @Override
  public void setSampleRateNs(long sampleRateNs) {}

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
    return this.pollingSubscription.shouldSample() ? 0 : Long.MAX_VALUE;
  }

  @Override
  public void sample() {
    // Getting new data point
    double distance = distanceSensor.getVoltage() * VOLTS_TO_INCHES_CONVERSION;
    distance = distance > 255 ? 255 : distance;
    distance = distance < 0 ? 0 : distance;
    dataPoints.add(new DataPoint(distance, System.nanoTime()));
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
  public void subscribe(PollingSubscription subscription) {
    this.pollingSubscription = subscription;
  }

  @Override
  public String getName() {
    return name;
  }
}
