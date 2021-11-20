package org.firstinspires.ftc.teamcode.cv;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.LinkedList;
import java.util.List;

public class TeamMarkerPositionDetector implements ITeamMarkerPositionDetector {
  @Override
  public TeamMarkerPosition calculateTeamMarkerPosition(Mat frame) {
    Mat cropped = new Mat(frame, new Rect(0, 0, frame.width(), frame.height() / 2));
    Mat hsvMat = cropped.clone();
    Imgproc.cvtColor(cropped, hsvMat, Imgproc.COLOR_RGB2HSV);
    Mat mask = hsvMat.clone();
    Mat kernel = Mat.ones(5, 5, CvType.CV_8UC1);
    Core.inRange(hsvMat, new Scalar(15, 35, 0), new Scalar(45, 150, 255), mask);
    Imgproc.erode(mask, mask, kernel);
    Imgproc.dilate(mask, mask, kernel);
    List<Integer> xPositions = new LinkedList<>();
    for (int i = 0; i < mask.rows(); i++) {
      for (int j = 0; j < mask.cols(); j++) {
        if (mask.get(i, j)[0] == 255) {
          xPositions.add(j);
        }
      }
    }
    double xLength = xPositions.size();
    double avgX = xPositions.stream().reduce(0, Integer::sum) / xLength;
    double third = (double) hsvMat.width() / 3.0;
    double middleThirdTop = third * 2;
    if (avgX > middleThirdTop) {
      return TeamMarkerPosition.RIGHT;
    }
    if (avgX <= middleThirdTop && avgX >= third) {
      return TeamMarkerPosition.CENTER;
    }
    return TeamMarkerPosition.LEFT;
  }
}
