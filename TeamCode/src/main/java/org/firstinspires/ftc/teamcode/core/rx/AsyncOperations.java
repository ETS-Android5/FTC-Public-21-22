package org.firstinspires.ftc.teamcode.core.rx;

import java.util.function.Function;
import java.util.function.Supplier;

public class AsyncOperations {
    static <T> void await(Supplier<T> func, Function<T, AsyncResolution> condition, long checkIntervalMs, Runnable task) {
        new Thread(() -> {
            T result = func.get();
            boolean canRunTask = false;
            while (!canRunTask) {
                switch (condition.apply(result)) {
                    case WAIT:
                        try {
                            Thread.sleep(checkIntervalMs);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        break;
                    case CONTINUE:
                        canRunTask = true;
                        break;
                    case CANCEL:
                        return;
                }
            }
            task.run();
        }).start();
    }
}
