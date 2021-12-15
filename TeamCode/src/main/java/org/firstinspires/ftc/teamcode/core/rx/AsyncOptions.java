package org.firstinspires.ftc.teamcode.core.rx;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

public class AsyncOptions<T> {
    private Supplier<T> intenseOperation;
    private Function<T, AsyncResolution> condition;
    private long conditionalIntervalMs = 250;
    private Consumer<T> postOp;
    private BiConsumer<T, Exception> catchTask;
    private Consumer<T> finallyTask;

    public Supplier<T> getIntenseOperation() {
        return intenseOperation;
    }

    public Function<T, AsyncResolution> getCondition() {
        return condition;
    }

    public long getConditionalIntervalMs() {
        return conditionalIntervalMs;
    }

    public Consumer<T> getPostOp() {
        return postOp;
    }

    public BiConsumer<T, Exception> getCatchTask() {
        return catchTask;
    }

    public Consumer<T> getFinallyTask() {
        return finallyTask;
    }

    public AsyncOptions<T> setIntenseOperation(Supplier<T> intenseOperation) {
        this.intenseOperation = intenseOperation;
        return this;
    }

    public AsyncOptions<T> setCondition(Function<T, AsyncResolution> condition) {
        this.condition = condition;
        return this;
    }

    public AsyncOptions<T> setConditionalIntervalMs(long conditionalIntervalMs) {
        this.conditionalIntervalMs = conditionalIntervalMs;
        return this;
    }

    public AsyncOptions<T> setPostOp(Consumer<T> postOp) {
        this.postOp = postOp;
        return this;
    }

    public AsyncOptions<T> setCatchTask(BiConsumer<T, Exception> catchTask) {
        this.catchTask = catchTask;
        return this;
    }

    public AsyncOptions<T> setFinallyTask(Consumer<T> finallyTask) {
        this.finallyTask = finallyTask;
        return this;
    }
}
