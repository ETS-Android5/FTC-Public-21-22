package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class PollingPipe extends HardwarePipeline implements PollingSubscription {
  private static final String POLL_TRIGGER_DEV_NAME = "POLL_TRIGGER";
  private static final byte POLLING_RATE = 50;

  private static PollingPipe instance;

  private final List<String> pollingTriggers = new LinkedList<>();
  private boolean allDataSourcesDiscovered = false;

  private long lastPollTime = 0;
  private boolean isMidPoll = false;
  private boolean shouldSample = false;

  public PollingPipe(String name, HardwarePipeline nextPipe) {
    super(name, nextPipe);
    PollingPipe.instance = this;
  }

  public static PollingPipe getInstance() {
    return instance;
  }

  @Override
  @SuppressWarnings("all")
  public StateFilterResult process(Map<String, Object> hardware, StateFilterResult r) {
    if (!allDataSourcesDiscovered) {
      pollingTriggers.clear();
      pollingTriggers.addAll(
          hardware.keySet().stream()
              .filter(s -> s.endsWith(POLL_TRIGGER_DEV_NAME))
              .collect(Collectors.toList()));
      if (InitializedFilterPipe.getInstance().everythingIsInitialized()) {
        allDataSourcesDiscovered = true;
      }
    }
    if (isMidPoll) {
      pollingTriggers.forEach(trigger -> ((DigitalChannel) hardware.get(trigger)).setState(false));
      lastPollTime = System.currentTimeMillis();
      isMidPoll = false;
      shouldSample = true;
    } else if (lastPollTime == 0 || System.currentTimeMillis() - lastPollTime <= POLLING_RATE) {
      pollingTriggers.forEach(trigger -> ((DigitalChannel) hardware.get(trigger)).setState(true));
      isMidPoll = true;
    }
    return super.process(hardware, r);
  }

  @Override
  public boolean shouldSample() {
    return shouldSample;
  }
}
