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

  public Component process(Map<String, Object> hardware, Component c) {
    if (nextElement != null) {
      Component res = nextElement.process(hardware, c);
      return res;
    } else {
      return c;
    }
  }

  public String getName() {
    return name;
  }
}
