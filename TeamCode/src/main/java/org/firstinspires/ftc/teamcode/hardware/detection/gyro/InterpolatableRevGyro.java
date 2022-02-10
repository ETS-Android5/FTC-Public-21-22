package org.firstinspires.ftc.teamcode.hardware.detection.gyro;

import com.google.common.collect.EvictingQueue;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.core.annotations.PostInit;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.PollingSubscription;
import org.firstinspires.ftc.teamcode.core.hardware.state.DataPoint;
import org.firstinspires.ftc.teamcode.core.hardware.state.Interpolatable;

@SuppressWarnings("UnstableApiUsage")
public class InterpolatableRevGyro implements Interpolatable {
  public static final String GYRO_NAME = "CB_AUTO_IMU";

  @Hardware(name = GYRO_NAME)
  public BNO055IMU gyro;

  private EvictingQueue<DataPoint> dataPoints;
  private long sampleRateNs;
  private int polynomialDegree;
  private long lastSampleTime;
  private boolean isSampling = false;
  private PollingSubscription pollingSubscription;

  public InterpolatableRevGyro(int analysisSize, long sampleRateNs, int polynomialDegree) {
    this.dataPoints = EvictingQueue.create(analysisSize);
    this.sampleRateNs = sampleRateNs;
    this.polynomialDegree = polynomialDegree;
    this.lastSampleTime = 0;
  }

  @PostInit(argType = BNO055IMU.class)
  @SuppressWarnings("unused")
  public void init(BNO055IMU arg) {
    initialize();
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
    synchronized (this) {
      dataPoints = nextQueue;
    }
  }

  @Override
  public long getSampleRateNs() {
    return sampleRateNs;
  }

  @Override
  public synchronized void setSampleRateNs(long sampleRateNs) {
    this.sampleRateNs = sampleRateNs;
  }

  @Override
  public int getPolynomialDegree() {
    return polynomialDegree;
  }

  @Override
  public synchronized void setPolynomialDegree(int polynomialDegree) {
    this.polynomialDegree = polynomialDegree;
  }

  @Override
  public long timeToNextSample() {
    if (pollingSubscription != null) {
      return pollingSubscription.shouldSample() ? 0 : Long.MAX_VALUE;
    }
    return lastSampleTime == 0 ? 0 : (lastSampleTime + sampleRateNs) - System.nanoTime();
  }

  @Override
  public void sample() {
    // Getting new data point
    double angle =
        gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES)
            .firstAngle;
    synchronized (this) {
      lastSampleTime = System.nanoTime();
    }
    dataPoints.add(new DataPoint(angle, lastSampleTime));
  }

  @Override
  public void initialize() {
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    gyro.initialize(parameters);
    gyro.startAccelerationIntegration(new Position(), new Velocity(), 20);
  }

  @Override
  public boolean isSampling() {
    return isSampling;
  }

  @Override
  public void startSampling() {
    dataPoints.clear();
    synchronized (this) {
      isSampling = true;
    }
  }

  @Override
  public void stopSampling() {
    dataPoints.clear();
    synchronized (this) {
      isSampling = false;
    }
  }

  @Override
  public synchronized void subscribe(PollingSubscription subscription) {
    pollingSubscription = subscription;
  }

  @Override
  public String getName() {
    return GYRO_NAME;
  }
}
