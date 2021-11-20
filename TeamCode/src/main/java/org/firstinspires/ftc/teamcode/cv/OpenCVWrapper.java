package org.firstinspires.ftc.teamcode.cv;

import org.opencv.android.OpenCVLoader;

public class OpenCVWrapper {
  private static boolean loaded = false;

  public static void load() {
    if (!loaded) {
      OpenCVLoader.initDebug();
      loaded = true;
    }
  }
}
