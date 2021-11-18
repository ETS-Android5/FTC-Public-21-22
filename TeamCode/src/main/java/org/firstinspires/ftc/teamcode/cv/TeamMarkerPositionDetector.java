package org.firstinspires.ftc.teamcode.cv;

import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class TeamMarkerPositionDetector implements ITeamMarkerPositionDetector {
  @Override
  public TeamMarkerPosition calculateTeamMarkerPosition(Mat frame) {
    Mat cropped = new Mat(frame, new Rect(0, 0, frame.width(), frame.height() / 2));
    Mat hsvMat = cropped.clone();
    Imgproc.cvtColor(cropped, hsvMat, Imgproc.COLOR_BGR2HSV);
    Mat mask = hsvMat.clone();
    Core.inRange(hsvMat, new Scalar(15, 20, 30), new Scalar(45, 110, 200), mask);
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2));
    Imgproc.erode(hsvMat, hsvMat, kernel);
    Imgproc.erode(hsvMat, hsvMat, kernel);
    Imgproc.dilate(hsvMat, hsvMat, kernel);
    Imgproc.dilate(hsvMat, hsvMat, kernel);
    List<Integer> xPositions = new LinkedList<>();
    for (int i = 0; i < hsvMat.width(); i++) {
      for (int j = 0; j < hsvMat.height(); j++) {
        double[] data = hsvMat.get(i, j);
        Log.d("VISION", Arrays.toString(data));
        if (data[0] == 1) {
          xPositions.add(j);
        }
      }
    }
    double xLength = xPositions.size();
    double avgX = xPositions.stream().reduce(0, Integer::sum) / xLength;
    double third = (double) hsvMat.width() / 3.0;
    double middleThirdTop = third * 2;
    Log.d("VISION", "X: " + avgX);
    if (avgX > middleThirdTop) {
      Log.d("VISION", "RIGHT");
      return TeamMarkerPosition.RIGHT;
    }
    if (avgX <= middleThirdTop && avgX >= third) {
      Log.d("VISION", "CENTER");
      return TeamMarkerPosition.CENTER;
    }
    Log.d("VISION", "LEFT");
    return TeamMarkerPosition.LEFT;
  }
}
