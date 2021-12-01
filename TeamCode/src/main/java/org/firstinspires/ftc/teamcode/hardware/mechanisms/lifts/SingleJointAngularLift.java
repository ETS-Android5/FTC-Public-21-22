package org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.MotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Collections;
import java.util.List;

public class SingleJointAngularLift implements ISingleJointAngularLift {
    private static final String LIFT_MOTOR_NAME = "LIFT_MOTOR";

    public SingleJointAngularLift() {
        initialize();
    }

    @Hardware(name = LIFT_MOTOR_NAME, runMode = RunMode.RUN_TO_POSITION, direction = Direction.FORWARD)
    public DcMotor liftMotor;

    private IMotorState liftMotorState;

    private void initialize() {
        liftMotorState =
                new MotorState(LIFT_MOTOR_NAME, false)
                        .withRunMode(RunMode.RUN_TO_POSITION)
                        .withTargetPosition(0)
                        .withPowerCurve(PowerCurves.RUN_TO_POSITION_ANGULAR_LIFT_CURVE);
    }

    @Override
    public String getName() {
        return this.getClass().getName();
    }

    @Override
    public List<? super State> getNextState() {
        return Collections.singletonList(liftMotorState.duplicate());
    }

    @Override
    public Integer getState() {
        return liftMotorState.getTargetPosition();
    }

    @Override
    public void goToPosition(int ticks) {
        liftMotorState = liftMotorState.withTargetPosition(ticks);
    }
}
