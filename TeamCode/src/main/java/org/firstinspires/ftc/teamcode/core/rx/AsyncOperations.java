package org.firstinspires.ftc.teamcode.core.rx;

public class AsyncOperations {
    static <T> void await (AsyncOptions<T> options) {
        new Thread(() -> {
            T result = null;
            try {
                result = options.getIntenseOperation().get();
                if (options.getCondition() != null) {
                    boolean canRunTask = false;
                    while (!canRunTask) {
                        switch (options.getCondition().apply(result)) {
                            case WAIT:
                                try {
                                    Thread.sleep(options.getConditionalIntervalMs());
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
                }
                options.getPostOp().accept(result);
            } catch (Exception e) {
                if (options.getCatchTask() != null) {
                    options.getCatchTask().accept(result, e);
                }
            } finally {
                if (options.getFinallyTask() != null) {
                    options.getFinallyTask().accept(result);
                }
            }
        }).start();
    }
}
