package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;
import java.util.Map;

public class BulkReadManagerPipe extends HardwarePipeline {
  private final List<LynxModule> lynxModules;
  private boolean initialized = false;

  public BulkReadManagerPipe(String name) {
    super(name);
    lynxModules = null;
  }

  public BulkReadManagerPipe(String name, HardwarePipeline nextPipe) {
    super(name, nextPipe);
    lynxModules = null;
  }

  public BulkReadManagerPipe(String name, List<LynxModule> lynxModules, HardwarePipeline nextPipe) {
    super(name, nextPipe);
    this.lynxModules = lynxModules;
  }

  @Override
  public StateFilterResult process(Map<String, Object> hardware, StateFilterResult r) {
    if (lynxModules != null) {
      if (!initialized) {
        lynxModules.forEach((l) -> l.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        initialized = true;
      }
      lynxModules.forEach(LynxModule::clearBulkCache);
    }
    return super.process(hardware, r);
  }
}
