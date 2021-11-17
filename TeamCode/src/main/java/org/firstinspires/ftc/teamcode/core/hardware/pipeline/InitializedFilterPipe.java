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

  private boolean allInitialized = false;

  @Override
  public Component process(Map<String, Object> hardware, Component c) {
    if (!allInitialized) {
      return super.process(
          hardware,
          new Component() {
            @Override
            public List<? super State> getNextState() {
                List<? super State> nextStateList = c.getNextState();
              List<? super State> initialized = nextStateList.stream()
                  .filter((s) -> hardware.containsKey(((State) s).getName()))
                  .collect(Collectors.toList());
              if (initialized.size() == nextStateList.size()) {
                  allInitialized = true;
              }
              return initialized;
            }

            @Override
            public String getName() {
              return c.getName();
            }
          });
    } else {
        return super.process(hardware, c);
    }
  }
}
