package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

public class DataTracker {
  public static <T> void evaluateCallbacks(
      List<CallbackData<T>> callbacks, List<CallbackData<T>> newCallbacks, Map<String, T> dataset) {
    Iterator<CallbackData<T>> iter = callbacks.iterator();
    while (iter.hasNext()) {
      CallbackData<T> data = iter.next();
      if (data == null) {
        iter.remove();
      } else {
        String name = data.getHardwareName();
        if (dataset.containsKey(name) && data.conditionsMet(dataset.get(name))) {
          data.getCallback().run();
          iter.remove();
          if (newCallbacks != null) {
            newCallbacks.remove(data);
          }
        }
      }
    }
  }

  public static <T, U> void evaluateCallbacks(
      List<CallbackData<T>> callbacks,
      List<CallbackData<T>> newCallbacks,
      Map<String, U> dataset,
      Function<U, T> transformer) {
    Map<String, T> transformedDataset = new HashMap<>();
    dataset.forEach((k, v) -> transformedDataset.put(k, transformer.apply(v)));
    evaluateCallbacks(callbacks, newCallbacks, transformedDataset);
  }
}
