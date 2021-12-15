package org.firstinspires.ftc.teamcode.hardware.detection.vision;

import org.opencv.core.Mat;

public interface FtcCamera {
  void init();

  void start();

  Mat grabFrame();

  void deinit();
}
