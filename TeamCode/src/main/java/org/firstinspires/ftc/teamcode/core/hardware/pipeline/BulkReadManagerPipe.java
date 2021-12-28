package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;
import java.util.Map;

public class BulkReadManagerPipe extends HardwarePipeline {
  private static BulkReadManagerPipe instance;
  private List<LynxModule> lynxModules;
  private boolean initialized = false;

  public BulkReadManagerPipe(String name) {
    super(name);
    BulkReadManagerPipe.instance = this;
  }

  public BulkReadManagerPipe(String name, HardwarePipeline nextPipe) {
    super(name, nextPipe);
    BulkReadManagerPipe.instance = this;
  }

  public static BulkReadManagerPipe getInstance() {
    return instance;
  }

  public void setLynxModules(List<LynxModule> lynxModules) {
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
