package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.game.related.Alliance;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.cv.CameraPosition;
import org.firstinspires.ftc.teamcode.cv.OpenCVWrapper;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPosition;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPositionDetector;
import org.firstinspires.ftc.teamcode.hardware.robots.TurretBot;

@Autonomous(name = "CB_AUTO_VisionTest")
public class VisionTest extends EnhancedAutonomous {

  private final TurretBot robot;

  public VisionTest() {
    super(new TurretBot(Alliance.RED));
    this.robot = (TurretBot) robotObject;
  }

  @Override
  public void onInitPressed() {
    OpenCVWrapper.load();
  }

  @Override
  public void onStartPressed() {
    long start = System.nanoTime();
    robot.webcam.start();
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
