package org.firstinspires.ftc.teamcode.core.rx;

import java.util.concurrent.BlockingQueue;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class AsyncOptions<T> {
    private Supplier<T> intenseOperation;
    private Consumer<T> preCondition;
    private BlockingQueue<Boolean> condition;
    private Consumer<T> postOp;
    private BiConsumer<T, Exception> catchTask;
    private Consumer<T> finallyTask;

    public Supplier<T> getIntenseOperation() {
        return intenseOperation;
    }

    public Consumer<T> getPreCondition() {
        return preCondition;
    }

    public BlockingQueue<Boolean> getCondition() {
        return condition;
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

    public AsyncOptions<T> setPreCondition(Consumer<T> preCondition) {
        this.preCondition = preCondition;
        return this;
    }

    public AsyncOptions<T> setCondition(BlockingQueue<Boolean> condition) {
        this.condition = condition;
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
