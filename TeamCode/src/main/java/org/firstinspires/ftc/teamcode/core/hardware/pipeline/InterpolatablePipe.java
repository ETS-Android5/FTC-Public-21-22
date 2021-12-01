package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import org.firstinspires.ftc.teamcode.core.hardware.state.DataPointEstimator;
import org.firstinspires.ftc.teamcode.core.hardware.state.Interpolatable;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class InterpolatablePipe extends HardwarePipeline {
    private static InterpolatablePipe instance;

    public static InterpolatablePipe getInstance() {
        return instance;
    }

    public InterpolatablePipe(String name) {
        super(name);
        InterpolatablePipe.instance = this;
    }

    public InterpolatablePipe(String name, HardwarePipeline nextElement) {
        super(name, nextElement);
        InterpolatablePipe.instance = this;
    }

    private boolean allDataSourcesDiscovered = false;

    Map<String, Interpolatable> trackedDataSources = new HashMap<>();

    @SuppressWarnings("All")
    public List<Double> currentDataPointOf(String hardwareDeviceName) {
        if (!trackedDataSources.containsKey(hardwareDeviceName)) {
            return null;
        }
        Interpolatable dataTracker = trackedDataSources.get(hardwareDeviceName);
        return DataPointEstimator.predictData(new ArrayList<>(dataTracker.getDataPoints()));
    }

    @Override
    public StateFilterResult process(Map<String, Object> hardware, StateFilterResult r) {
        if (!allDataSourcesDiscovered) {
            hardware.forEach((k, v) -> {
                if (!trackedDataSources.containsKey(k) && v instanceof Interpolatable) {
                    trackedDataSources.put(k, (Interpolatable) v);
                }
            });
            if (InitializedFilterPipe.getInstance().everythingIsInitialized()) {
                allDataSourcesDiscovered = true;
            }
        }
        trackedDataSources.values().forEach(d -> {
            if (d.timeToNextSample() < 0) {
                d.sample();
            }
        });
        return super.process(hardware, r);
    }
}
