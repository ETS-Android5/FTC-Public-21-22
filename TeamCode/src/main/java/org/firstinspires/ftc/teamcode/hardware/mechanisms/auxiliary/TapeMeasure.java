package org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.hardware.state.IServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.ServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Arrays;
import java.util.List;

import kotlin.Triple;

public class TapeMeasure implements ITapeMeasure {
    private static final String YAW_SERVO_NAME = "TAPE_MEASURE_YAW_SERVO";
    private static final String PITCH_SERVO_NAME = "TAPE_MEASURE_PITCH_SERVO";
    private static final String LENGTH_SERVO_NAME = "TAPE_MEASURE_LENGTH_SERVO";
    private static final double SERVO_INIT_SPEED = 0.5; // 0 is full reverse and 1 is full forwards

    @Hardware(name = YAW_SERVO_NAME)
    public Servo yawServo;
    private IServoState yawServoState;

    @Hardware(name = YAW_SERVO_NAME)
    public Servo pitchServo;
    private IServoState pitchServoState;

    @Hardware(name = YAW_SERVO_NAME)
    public Servo lengthServo;
    private IServoState lengthServoState;

    public TapeMeasure() {
        initialize();
    }

    private void initialize() {
        yawServoState = new ServoState(YAW_SERVO_NAME, Direction.FORWARD, SERVO_INIT_SPEED);
        pitchServoState = new ServoState(PITCH_SERVO_NAME, Direction.FORWARD, SERVO_INIT_SPEED);
        lengthServoState = new ServoState(LENGTH_SERVO_NAME, Direction.FORWARD, SERVO_INIT_SPEED);
    }

    @Override
    public String getName() {
        return this.getClass().getName();
    }

    @Override
    public List<? super State> getNextState() {
        return Arrays.asList(yawServoState, pitchServoState, lengthServoState);
    }

    @Override
    public Triple<Double, Double, Double> getState() {
        return new Triple<>(
                yawServoState.getPosition(),
                pitchServoState.getPosition(),
                lengthServoState.getPosition());
    }

    @Override
    public synchronized void setYawRate(double amt) {
        yawServoState = yawServoState.withPosition(centerStickValueForServo(amt));
    }

    @Override
    public synchronized void setPitchRate(double amt) {
        pitchServoState = pitchServoState.withPosition(centerStickValueForServo(amt));
    }

    @Override
    public synchronized void setLengthRate(double amt) {
        lengthServoState = lengthServoState.withPosition(centerStickValueForServo(amt));
    }

    private double centerStickValueForServo(double stickValue) {
        return (stickValue / 2.0) + 0.5;
    }
}
