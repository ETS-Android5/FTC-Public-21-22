package org.firstinspires.ftc.teamcode.cv;

import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.LinkedList;
import java.util.List;

public class TeamMarkerPositionDetector implements ITeamMarkerPositionDetector {
  @Override
  public TeamMarkerPosition calculateTeamMarkerPosition(
      Mat frame, CameraPosition position, ViewPortDescription viewPortDescription) {
    // Resize the image to a fraction of the original resolution, because we don't need high res
    Mat resized = new Mat();
    Imgproc.resize(
        frame,
        resized,
        new Size((int) Math.round(frame.width() / 10.0), (int) Math.round(frame.height() / 10.0)),
        .1,
        .1,
        Imgproc.INTER_AREA);
    // Crop out the useless data, because we only need the horizontal area containing the marker
    Rect crop;
    if (position == CameraPosition.REAR_LOW_AND_CENTERED) {
      crop =
          new Rect(
              0,
              (int) Math.round(resized.height() / 2.1),
              resized.width(),
              (int)
                  Math.round(
                      resized.height() - (resized.height() / 2.1) - (resized.height() / 4.7)));
    } else {
      crop = new Rect(0, 0, resized.width(), resized.height() / 2);
    }
    Mat cropped = new Mat(resized, crop);
    // Convert to hsv for reliability in different lighting scenarios
    Mat hsvMat = new Mat();
    Imgproc.cvtColor(cropped, hsvMat, Imgproc.COLOR_RGB2HSV);
    // Create a binary image highlighting the areas of the image containing the marker color
    Mat mask = new Mat();
    Core.inRange(hsvMat, new Scalar(16, 90, 40), new Scalar(26, 180, 200), mask);
    // Remove noise by eroding and dilating the image with this kernel
    Mat kernel = Mat.ones(1, 1, CvType.CV_8UC1);
    Imgproc.erode(mask, mask, kernel);
    Imgproc.dilate(mask, mask, kernel);
    // Find the average x coordinate and return a marker position based off of it
    List<Integer> xPositions = new LinkedList<>();
    for (int i = 0; i < mask.rows(); i++) {
      for (int j = 0; j < mask.cols(); j++) {
        if (mask.get(i, j)[0] == 255) {
          xPositions.add(j);
        }
      }
    }
    Log.d("CV", "Discovered " + xPositions.size() + " matching pixels");
    return findTeamMarker(xPositions, mask.width(), viewPortDescription);
  }

  private TeamMarkerPosition findTeamMarker(
      List<Integer> xPositions, int width, ViewPortDescription viewPortDescription) {
    double xLength = xPositions.size();
    double avgX = xLength > 0 ? xPositions.stream().reduce(0, Integer::sum) / xLength : 0;
    double third;
    double half;
    switch (viewPortDescription) {
      case THREE_IN_VIEW:
        third = (double) width / 3.0;
        double middleThirdTop = third * 2;
        if (avgX < third) {
          return TeamMarkerPosition.LEFT;
        }
        if (avgX >= third && avgX <= middleThirdTop) {
          return TeamMarkerPosition.CENTER;
        }
        return TeamMarkerPosition.RIGHT;
      case LEFT_TWO_IN_VIEW:
        half = (double) width / 2.0;
        if (xLength > 0) {
          return avgX < half ? TeamMarkerPosition.LEFT : TeamMarkerPosition.CENTER;
        }
        return TeamMarkerPosition.RIGHT;
      case RIGHT_TWO_IN_VIEW:
        half = (double) width / 2.0;
        if (xLength > 0) {
          return avgX < half ? TeamMarkerPosition.CENTER : TeamMarkerPosition.RIGHT;
        }
        return TeamMarkerPosition.LEFT;
    }
    return TeamMarkerPosition.RIGHT;
  }
}
