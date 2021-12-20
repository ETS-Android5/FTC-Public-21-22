package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.cv.CameraPosition;
import org.firstinspires.ftc.teamcode.cv.OpenCVWrapper;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPosition;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPositionDetector;
import org.firstinspires.ftc.teamcode.hardware.robots.NewChassis;

import java.io.PrintWriter;
import java.io.StringWriter;

@Autonomous(name = "CB_AUTO_VisionTest")
public class VisionTest extends EnhancedAutonomous {

  private final NewChassis robot;

  public VisionTest() {
    super(new NewChassis());
    this.robot = (NewChassis) robotObject;
  }

  @Override
  public void onInitPressed() {
    OpenCVWrapper.load();
  }

  @Override
  public void onStartPressed() {
    long start = System.nanoTime();
    Thread worker =
        new Thread(
            () -> {
              robot.webcam.init();
              robot.webcam.start();
            });
    worker.start();
    try {
      worker.join();
    } catch (InterruptedException e) {
      StringWriter sw = new StringWriter();
      PrintWriter pw = new PrintWriter(sw);
      e.printStackTrace(pw);
    }
    TeamMarkerPosition teamMarkerPosition =
        new TeamMarkerPositionDetector()
            .calculateTeamMarkerPosition(
                robot.webcam.grabFrame(), CameraPosition.REAR_LOW_AND_CENTERED);
    long end = System.nanoTime();
    Log.d("TIME", "" + (end - start));
    Log.d("TIME", teamMarkerPosition == null ? "NULL" : teamMarkerPosition.name());
  }

  @Override
  public void onStop() {}
}
