package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;

import java.util.List;

public class DataPointEstimator {
  public static double predictData(List<DataPoint> srcData, long timestamp, int d) {
    return generatePolynomials(srcData, d).value(timestamp);
  }

  public static PolynomialFunction generatePolynomials(List<DataPoint> srcData, int d) {
    WeightedObservedPoints points = new WeightedObservedPoints();
    srcData.forEach((p) -> points.add(p.getTimestamp(), p.getData()));
    return new PolynomialFunction(PolynomialCurveFitter.create(d).fit(points.toList()));
  }
}
