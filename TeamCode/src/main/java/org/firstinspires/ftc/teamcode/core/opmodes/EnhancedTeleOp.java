package org.firstinspires.ftc.teamcode.core.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.core.controller.Controller;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.ExitPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.HardwarePipeline;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.InitializedFilterPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.MotorTrackerPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.ResetDcMotorPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.RunToPositionPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.StateFilterResult;
import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;
import org.firstinspires.ftc.teamcode.core.magic.runtime.AotRuntime;
import org.firstinspires.ftc.teamcode.core.magic.runtime.HardwareMapDependentReflectionBasedMagicRuntime;
import org.firstinspires.ftc.teamcode.core.magic.runtime.ReflectionBasedMagicRuntime;
import org.firstinspires.ftc.teamcode.core.magic.runtime.ServiceRuntime;

import java.util.concurrent.ConcurrentHashMap;

public abstract class EnhancedTeleOp extends OpMode {

  protected Controller controller1;
  protected Controller controller2;
  protected final ConcurrentHashMap<String, Object> initializedHardware;
  protected final HardwarePipeline hardwarePipeline;
  private HardwareMapDependentReflectionBasedMagicRuntime aotRuntime;
  private ReflectionBasedMagicRuntime serviceRuntime;
  protected Component robotObject;

  public EnhancedTeleOp(Component robotObject) {
    State.clear();
    initializedHardware = new ConcurrentHashMap<>();
    hardwarePipeline =
        new HardwarePipeline(
            Constants.PIPELINE_BASE_NAME,
            new InitializedFilterPipe(
                "FilterElement",
                new ResetDcMotorPipe(
                    "MotorReset",
                    new MotorTrackerPipe(
                        "MotorTrackerPipe",
                        new RunToPositionPipe("RunToPosition", new ExitPipe("Exit"))))));
    controller1 = new Controller(Constants.GAMEPAD_1_NAME);
    controller2 = new Controller(Constants.GAMEPAD_2_NAME);
    this.robotObject = robotObject;
    initialize();
  }

  protected void initialize() {
    aotRuntime = new AotRuntime(robotObject, initializedHardware, false);
    aotRuntime.initialize();
    serviceRuntime = new ServiceRuntime(telemetry, robotObject, controller1, controller2);
    serviceRuntime.initialize();
  }

  @Override
  public final void init() {
    telemetry.setMsTransmissionInterval(Constants.TELEMETRY_TRANSMISSION_INTERVAL);
    aotRuntime.setHardwareMap(hardwareMap);
    aotRuntime.waveWand();

    serviceRuntime.waveWand();
    onInitPressed();
  }

  public abstract void onInitPressed();

  @Override
  public final void init_loop() {
    super.init_loop();
    initLoop();
  }

  public abstract void initLoop();

  @Override
  public final void start() {
    super.start();
    controller1.initialize(gamepad1);
    controller2.initialize(gamepad2);
    onStartPressed();
  }

  public abstract void onStartPressed();

  @Override
  public final void loop() {
    controller1.update();
    controller2.update();
    onLoop();
    hardwarePipeline.process(initializedHardware, new StateFilterResult(robotObject));
  }

  public abstract void onLoop();

  @Override
  public final void stop() {
    super.stop();
    onStop();
    State.clear();
  }

  public abstract void onStop();
}
