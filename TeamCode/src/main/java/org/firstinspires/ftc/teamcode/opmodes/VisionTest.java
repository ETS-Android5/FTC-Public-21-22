package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.core.rx.AsyncOperations;
import org.firstinspires.ftc.teamcode.core.rx.AsyncTaskDescription;
import org.firstinspires.ftc.teamcode.cv.OpenCVWrapper;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPosition;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPositionDetector;
import org.firstinspires.ftc.teamcode.hardware.robots.NewChassis;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

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
    robot.webcam.init();
    long start = System.nanoTime();
    robot.webcam.start();
    BlockingQueue<TeamMarkerPosition> markerPositionQueue = new LinkedBlockingQueue<>(1);
    AsyncTaskDescription<Void> asyncTask =
        new AsyncTaskDescription<Void>()
            .setIntenseOperation(
                () -> {
                  markerPositionQueue.add(
                      new TeamMarkerPositionDetector()
                          .calculateTeamMarkerPosition(robot.webcam.grabFrame()));
                  return null;
                })
            .setCatchTask((u1, u2) -> markerPositionQueue.add(null))
            .setFinallyTask((unused) -> robot.webcam.deinit());
    AsyncOperations.startAsyncTask(asyncTask);
    TeamMarkerPosition teamMarkerPosition = null;
    try {
      teamMarkerPosition = markerPositionQueue.take();
    } catch (InterruptedException ignored) {
    }
    long end = System.nanoTime();
    Log.d("TIME", "" + (end - start));
    Log.d("TIME", teamMarkerPosition == null ? "NULL" : teamMarkerPosition.name());
  }

  @Override
  public void onStop() {}
}
