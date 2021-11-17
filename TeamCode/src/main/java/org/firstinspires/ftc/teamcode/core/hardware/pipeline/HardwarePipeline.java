package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;

import java.util.Map;

public class HardwarePipeline {
  final String name;
  final HardwarePipeline nextElement;

  public HardwarePipeline(String name) {
    this.name = name;
    this.nextElement = null;
  }

  public HardwarePipeline(String name, HardwarePipeline nextElement) {
    this.name = name;
    this.nextElement = nextElement;
  }

  public StateFilterResult process(Map<String, Object> hardware, StateFilterResult r) {
    if (nextElement != null) {
      return nextElement.process(hardware, r);
    } else {
      return r;
    }
  }

  public String getName() {
    return name;
  }
}
