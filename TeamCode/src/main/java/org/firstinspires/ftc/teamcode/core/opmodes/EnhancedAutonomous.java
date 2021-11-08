package org.firstinspires.ftc.teamcode.core.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.hardware.pipeline.HardwarePipeline;
import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;
import org.firstinspires.ftc.teamcode.core.magic.runtime.AotRuntime;
import org.firstinspires.ftc.teamcode.core.magic.runtime.ReflectionBasedMagicRuntime;
import org.firstinspires.ftc.teamcode.core.magic.runtime.ServiceRuntime;

import java.util.concurrent.ConcurrentHashMap;

public abstract class EnhancedAutonomous extends LinearOpMode {

  protected final ConcurrentHashMap<String, Object> initializedHardware;
  protected final HardwarePipeline hardwarePipeline;
  private ReflectionBasedMagicRuntime aotRuntime;
  private ReflectionBasedMagicRuntime serviceRuntime;
  private Component robotObject;

  public EnhancedAutonomous(HardwarePipeline pipeline) {
    initializedHardware = new ConcurrentHashMap<>();
    hardwarePipeline = pipeline;
  }

  protected void initialize(Component robotObject) {
    this.robotObject = robotObject;
    aotRuntime =
            new AotRuntime(
                    robotObject,
                    initializedHardware);
    aotRuntime.initialize();
    serviceRuntime = new ServiceRuntime(telemetry, robotObject);
    serviceRuntime.initialize();
  }

  @Override
  public final void runOpMode() {
    telemetry.setMsTransmissionInterval(Constants.TELEMETRY_TRANSMISSION_INTERVAL);
    waitForStart();
    onInitPressed();
    aotRuntime.waveWand();
    serviceRuntime.waveWand();
    onStartPressed();
    State.clear();
  }

  public abstract void onInitPressed();

  public abstract void onStartPressed();
}
