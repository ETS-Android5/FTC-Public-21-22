package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class InitializedFilterPipe extends HardwarePipeline {
  public InitializedFilterPipe(String name) {
    super(name);
  }

  public InitializedFilterPipe(String name, HardwarePipeline nextPipe) {
    super(name, nextPipe);
  }

  @Override
  public Component process(Map<String, Object> hardware, Component c) {
    return super.process(
        hardware,
        new Component() {
          @Override
          public List<? super State> getNextState() {
            return c.getNextState().stream()
                .filter((s) -> hardware.containsKey(((State) s).getName()))
                .collect(Collectors.toList());
          }

          @Override
          public String getName() {
            return c.getName();
          }
        });
  }
}
