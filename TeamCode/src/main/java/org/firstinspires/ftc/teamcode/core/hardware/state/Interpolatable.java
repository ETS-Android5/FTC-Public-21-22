package org.firstinspires.ftc.teamcode.core.hardware.state;

import com.google.common.collect.EvictingQueue;

import org.firstinspires.ftc.teamcode.core.Namable;

public interface Interpolatable extends Namable {
  @SuppressWarnings("UnstableApiUsage")
  EvictingQueue<DataPoint> getDataPoints();

  int getAnalysisSize();

  void setAnalysisSize(int size);

  long getSampleRateNs();

  void setSampleRateNs(long sampleRateNs);

  int getPolynomialDegree();

  void setPolynomialDegree(int polynomialDegree);

  long timeToNextSample();

  void sample();

  void initialize();

  boolean isSampling();

  void startSampling();

  void stopSampling();
}
