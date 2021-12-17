package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import org.firstinspires.ftc.teamcode.core.hardware.state.CachedValue;
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
    if (!trackedDataSources.containsKey(hardwareDeviceName)) {
      return null;
    }
    CachedValue<Double> cachedValue = cachedValues.get(hardwareDeviceName);
    if (cachedValue == null || cachedValue.getId() != pipelineIteration) {
      Interpolatable dataTracker = trackedDataSources.get(hardwareDeviceName);
      double val =
          DataPointEstimator.predictData(
              new LinkedList<>(dataTracker.getDataPoints()),
              System.nanoTime(),
              dataTracker.getPolynomialDegree());
      cachedValues.put(hardwareDeviceName, new CachedValue<>(val, pipelineIteration));
      return val;
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
        trackedDataSources,
        (Interpolatable i) -> currentDataPointOf(i.getName()));
    inPipe = false;
    return super.process(hardware, r);
  }
}
