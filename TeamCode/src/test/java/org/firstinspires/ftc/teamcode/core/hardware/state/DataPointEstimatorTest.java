package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.junit.jupiter.api.Test;

import java.util.LinkedList;
import java.util.List;

public class DataPointEstimatorTest {
    @Test
    public void predictDataTest() {
        List<List<DataPoint>> testData = new LinkedList<>();
        // Data set: 1, 4, 15, 40, 85
        // Expected prediction: 156
        List<DataPoint> e1 = new LinkedList<>();
        e1.add(new DataPoint(1.0, 0));
        e1.add(new DataPoint(4.0, 1));
        e1.add(new DataPoint(15.0, 2));
        e1.add(new DataPoint(40.0, 3));
        e1.add(new DataPoint(85.0, 4));
        testData.add(e1);
        org.junit.jupiter.api.Assertions.assertEquals((double) DataPointEstimator.predictData(testData).get(0), 156);
    }
}
