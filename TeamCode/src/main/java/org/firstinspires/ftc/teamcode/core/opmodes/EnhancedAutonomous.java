package org.firstinspires.ftc.teamcode.core.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.hardware.pipeline.BulkReadManagerPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.ExitPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.HardwarePipeline;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.InitializedFilterPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.InterpolatablePipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.MotorTrackerPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.PollingPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.RunToPositionPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.StateFilterResult;
import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;
import org.firstinspires.ftc.teamcode.core.magic.runtime.AotRuntime;
import org.firstinspires.ftc.teamcode.core.magic.runtime.HardwareMapDependentReflectionBasedMagicRuntime;
import org.firstinspires.ftc.teamcode.core.magic.runtime.ReflectionBasedMagicRuntime;
import org.firstinspires.ftc.teamcode.core.magic.runtime.ServiceRuntime;

import java.util.concurrent.ConcurrentHashMap;

public abstract class EnhancedAutonomous extends LinearOpMode {

  protected final ConcurrentHashMap<String, Object> initializedHardware;
  protected final HardwarePipeline hardwarePipeline;
  protected Component robotObject;
  private HardwareMapDependentReflectionBasedMagicRuntime aotRuntime;
  private ReflectionBasedMagicRuntime serviceRuntime;

  public EnhancedAutonomous(Component robotObject) {
    State.clear();
    initializedHardware = new ConcurrentHashMap<>();
    hardwarePipeline =
        new HardwarePipeline(
            Constants.PIPELINE_BASE_NAME,
            new BulkReadManagerPipe(
                "BulkReadManager",
                new InitializedFilterPipe(
                    "FilterElement",
                    new PollingPipe(
                        "Polling",
                        new InterpolatablePipe(
                            "Interpolatable",
                            new MotorTrackerPipe(
                                "MotorTracker",
                                new RunToPositionPipe("RunToPosition", new ExitPipe("Exit"))))))));
    this.robotObject = robotObject;
    initialize();
  }

  private void initialize() {
    aotRuntime = new AotRuntime(robotObject, initializedHardware, true);
    aotRuntime.initialize();
    serviceRuntime = new ServiceRuntime(telemetry, robotObject, this);
    serviceRuntime.initialize();
  }

  protected void processChanges() {
    hardwarePipeline.process(initializedHardware, new StateFilterResult(robotObject));
    telemetry.update();
  }

  @Override
  public final void runOpMode() {
    telemetry.setMsTransmissionInterval(Constants.TELEMETRY_TRANSMISSION_INTERVAL);
    aotRuntime.setHardwareMap(hardwareMap);
    BulkReadManagerPipe.getInstance().setLynxModules(hardwareMap.getAll(LynxModule.class));
    aotRuntime.waveWand();
    serviceRuntime.waveWand();
    onInitPressed();
    waitForStart();
    if (!isStopRequested()) {
      onStartPressed();
    }
    onStop();
    State.clear();
  }

  public abstract void onInitPressed();

  public abstract void onStartPressed();

  public abstract void onStop();

  protected void wait(int ms) {
    long start = System.currentTimeMillis();
    while (System.currentTimeMillis() - start < ms && opModeIsActive()) {
      processChanges();
    }
  }
}
