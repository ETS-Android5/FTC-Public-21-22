package org.firstinspires.ftc.teamcode.core.hardware.state;

import java.util.LinkedList;
import java.util.List;

public class DataPointEstimator {
    public static List<Double> predictData(List<List<DataPoint>> srcData) {
        int minDim = srcData.get(0).size();
        // Make sure our data has the same number of variables all the way through
        for (List<DataPoint> dataset : srcData) {
            if (dataset.size() != minDim) return null;
        }
        // Copy our data to an array for faster access and manipulation
        DataPoint[][] dataPointsTemp = new DataPoint[srcData.size()][minDim];
        int idx = 0;
        for (List<DataPoint> srcDatum : srcData) {
            dataPointsTemp[idx] = srcDatum.toArray(new DataPoint[0]);
            idx++;
        }
        // Array dimension one is position in the queue from 0 to n with 0 most recent and n oldest
        // Array dimension two is the channel of data
        // For example, a color sensor may give 3 data points (R, G, B) in a single reading
        // So the second dimension would have a length of 3

        // Let's restructure the array so that the first dimension is the data channel and the second is the position in the queue
        DataPoint[][] dataPoints = new DataPoint[minDim][dataPointsTemp.length];
        for (int i = 0; i < minDim; i++) {
            for (int j = 0; j < dataPointsTemp.length; j++) {
                dataPoints[i][j] = dataPointsTemp[j][i];
            }
        }

        // TODO: Consolidate the above stuff into a single nested for loop rather than two of them

        // Each channel gets its own depth map and rate of change
        // The list is for all the data channels
        // The depth map is the rate of change as a double (change per microsecond)
        // The depth is based on  how many times rate of change can be derived
        // Depth is up n-1 where n is the number of data points
        // A depth map for the following dataset: 1, 4, 15, 40, 85 looks like this
        // Depth    Data points
        // 1        3, 11, 25, 45
        // 2        8, 14, 20
        // 3        6, 6
        // 4        0

        // Prediction tree
        // 0, 0
        // 6, 6, 6
        // 8, 14, 20, 26
        // 3, 11, 25, 45, 71
        // 1, 4, 15, 40, 85, 156
        List<Double> predictions = new LinkedList<>();
        for (DataPoint[] dataPoint : dataPoints) {
            predictions.add(makePrediction(generateDepthBasedRatesOfChange(dataPoint), dataPoint[dataPoint.length - 1].getData()));
        }
        return predictions;
    }

    private static double[][] generateDepthBasedRatesOfChange(DataPoint[] dataSrc) {
        switch (dataSrc.length) {
            case 0:
                return null;
            case 1:
                return new double[][] { { dataSrc[0].getData() } };
            case 2:
                return new double[][] { { (dataSrc[1].getData() - dataSrc[0].getData()) /
                        (dataSrc[1].getTimestamp() - dataSrc[0].getTimestamp()) } };
            default:
                double[][] ratesOfChange = new double[dataSrc.length - 1][dataSrc.length - 1];

                for (int i = 0; i < ratesOfChange.length; i++) {
                    ratesOfChange[0][i] = (dataSrc[i + 1].getData() - dataSrc[i].getData()) /
                            (dataSrc[i + 1].getTimestamp() - dataSrc[i].getTimestamp());
                }
                for (int i = 1; i < ratesOfChange.length; i++) {
                    for (int j = 0; j < ratesOfChange.length - i; j++) {
                        ratesOfChange[i][j] = ratesOfChange[i - 1][j + 1] - ratesOfChange[i - 1][j];
                    }
                }
                return ratesOfChange;
        }

    }

    private static double makePrediction(double[][] ratesOfChange, double mostRecent) {
        if (ratesOfChange == null) return 0;
        double prediction = mostRecent;
        for (int i = 0; i < ratesOfChange.length; i++) {
            prediction += ratesOfChange[ratesOfChange.length - (1 + i)][i];
        }
        return prediction;
    }
}