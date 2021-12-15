package org.firstinspires.ftc.teamcode.core.rx;

public class AsyncOperations {
  public static <T> void startAsyncTask(AsyncTaskDescription<T> options) {
    new Thread(
            () -> {
              T result = null;
              try {
                if (options.getIntenseOperation() != null) {
                  result = options.getIntenseOperation().get();
                }
                if (options.getPreCondition() != null) {
                  options.getPreCondition().accept(result);
                }
                if (options.getCondition() != null) {
                  Boolean resolution = options.getCondition().take();
                  if (resolution && options.getPostOp() != null) {
                    options.getPostOp().accept(result);
                  }
                } else if (options.getPostOp() != null) {
                  options.getPostOp().accept(result);
                }
              } catch (Exception e) {
                if (options.getCatchTask() != null) {
                  options.getCatchTask().accept(result, e);
                }
              } finally {
                if (options.getFinallyTask() != null) {
                  options.getFinallyTask().accept(result);
                }
              }
            })
        .start();
  }
}
