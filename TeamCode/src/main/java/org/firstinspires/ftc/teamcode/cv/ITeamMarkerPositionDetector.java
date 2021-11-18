package org.firstinspires.ftc.teamcode.cv;

import org.opencv.core.Mat;

public interface ITeamMarkerPositionDetector {
  TeamMarkerPosition calculateTeamMarkerPosition(Mat frame);
}
