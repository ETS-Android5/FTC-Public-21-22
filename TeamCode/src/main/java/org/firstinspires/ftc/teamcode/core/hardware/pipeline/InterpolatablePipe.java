package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import org.firstinspires.ftc.teamcode.core.hardware.state.CachedValue;
import org.firstinspires.ftc.teamcode.core.hardware.state.DataPoint;
import org.firstinspires.ftc.teamcode.core.hardware.state.DataPointEstimator;
import org.firstinspires.ftc.teamcode.core.hardware.state.Interpolatable;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class InterpolatablePipe extends HardwarePipeline {
  private static InterpolatablePipe instance;
  private final Map<String, Interpolatable> trackedDataSources = new HashMap<>();
  private final Map<String, CachedValue<Double>> cachedValues = new HashMap<>();
  private final List<CallbackData<Double>> dataSourceCallbacks = new LinkedList<>();
  private boolean allDataSourcesDiscovered = false;
  private long pipelineIteration = 0;
  private boolean inPipe = false;

  public InterpolatablePipe(String name) {
    super(name);
    InterpolatablePipe.instance = this;
  }

  public InterpolatablePipe(String name, HardwarePipeline nextElement) {
    super(name, nextElement);
    InterpolatablePipe.instance = this;
  }

  public static InterpolatablePipe getInstance() {
    return instance;
  }

  public void setCallbackForInterpolatable(CallbackData<Double> data) {
    if (!inPipe) {
      dataSourceCallbacks.add(data);
    } else {
      ExitPipe.getInstance().onNextTick(() -> dataSourceCallbacks.add(data));
    }
  }

  @SuppressWarnings("All")
  public Double currentDataPointOf(String hardwareDeviceName) {
    long currentTime = System.nanoTime();
    if (!trackedDataSources.containsKey(hardwareDeviceName)) {
      return null;
    }
    CachedValue<Double> cachedValue = cachedValues.get(hardwareDeviceName);
    if (cachedValue == null || cachedValue.getId() != pipelineIteration) {
      Interpolatable dataTracker = trackedDataSources.get(hardwareDeviceName);
      LinkedList<DataPoint> dataPoints = new LinkedList<>(dataTracker.getDataPoints());
      Double ret = null;
      switch (dataPoints.size()) {
        case 0:
          ret = null;
          break;
        case 1:
          ret = dataPoints.getFirst().getData();
          break;
        case 2:
          DataPoint pt1 = dataPoints.getFirst();
          DataPoint pt2 = dataPoints.get(1);
          double rateOfChangeUnitPerNanosecond = (pt2.getData() - pt1.getData()) /
                  (pt2.getTimestamp() - pt1.getTimestamp());
          double yIntercept = pt1.getData() - (pt1.getTimestamp() * rateOfChangeUnitPerNanosecond);
          ret = (currentTime * rateOfChangeUnitPerNanosecond) + yIntercept;
          break;
        default:
          ret = DataPointEstimator.predictData(
                  new LinkedList<>(dataTracker.getDataPoints()),
                  currentTime,
                  dataTracker.getPolynomialDegree());
          break;
      }
      cachedValues.put(hardwareDeviceName, new CachedValue<>(ret, pipelineIteration));
      return ret;
    }
    return cachedValue.getValue();
  }

  @Override
  public StateFilterResult process(Map<String, Object> hardware, StateFilterResult r) {
    inPipe = true;
    pipelineIteration++;
    if (!allDataSourcesDiscovered) {
      hardware.forEach(
          (k, v) -> {
            if (!trackedDataSources.containsKey(k) && v instanceof Interpolatable) {
              trackedDataSources.put(k, (Interpolatable) v);
            }
          });
      if (InitializedFilterPipe.getInstance().everythingIsInitialized()) {
        allDataSourcesDiscovered = true;
      }
    }
    trackedDataSources
        .values()
        .forEach(
            d -> {
              if (d.isSampling() && d.timeToNextSample() <= 0) {
                d.sample();
              }
            });
    DataTracker.evaluateCallbacks(
        dataSourceCallbacks,
        null,
        trackedDataSources,
        (Interpolatable i) -> currentDataPointOf(i.getName()));
    inPipe = false;
    return super.process(hardware, r);
  }
}
