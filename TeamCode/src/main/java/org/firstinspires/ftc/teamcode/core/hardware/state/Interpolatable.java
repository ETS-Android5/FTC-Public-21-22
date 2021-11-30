package org.firstinspires.ftc.teamcode.core.hardware.state;

import com.google.common.collect.EvictingQueue;

import java.util.List;

public interface Interpolatable {
    @SuppressWarnings("UnstableApiUsage")
    EvictingQueue<List<DataPoint>> getDataPoints();
    int getAnalysisSize();
    void setAnalysisSize(int size);
    long getSampleRateNs();
    void setSampleRateNs(long sampleRateNs);
    long timeToNextSample();
    void sample();
    void forceSample();
}
