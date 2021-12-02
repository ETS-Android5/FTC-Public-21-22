package org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.hardware.state.IServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.ServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Collections;
import java.util.List;

public class SingleServoGripper implements ISingleServoGripper {
    private static final String SINGLE_SERVO_GRIPPER_SERVO_NAME = "SINGLE_SERVO_GRIPPER_SERVO";

    private static final double OPEN_POSITION = 0.4;
    private static final double CLOSED_POSITION = 0.2;

    public SingleServoGripper() {
        initialize();
    }

    @Hardware(name = SINGLE_SERVO_GRIPPER_SERVO_NAME)
    public Servo singleServoGripperServo;

    private IServoState singleServoGripperServoState;

    private void initialize() {
        singleServoGripperServoState =
                new ServoState(SINGLE_SERVO_GRIPPER_SERVO_NAME, Direction.FORWARD, OPEN_POSITION);
    }

    @Override
    public String getName() {
        return this.getClass().getName();
    }

    @Override
    public List<? super State> getNextState() {
        return Collections.singletonList(singleServoGripperServoState);
    }

    @Override
    public SingleServoGripperState getState() {
        return singleServoGripperServoState.getPosition() == CLOSED_POSITION ? SingleServoGripperState.CLOSED_POSITION : SingleServoGripperState.OPEN_POSITION;
    }

    @Override
    public void open() {
        singleServoGripperServoState = singleServoGripperServoState.withPosition(OPEN_POSITION);
    }

    @Override
    public void close() {
        singleServoGripperServoState = singleServoGripperServoState.withPosition(CLOSED_POSITION);
    }
}
