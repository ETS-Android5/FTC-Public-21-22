package org.firstinspires.ftc.teamcode.hardware.mechanisms.intakes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.annotations.Observable;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.IServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.MotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.ServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Arrays;
import java.util.List;

public class Intake implements IIntake {
    private static final String INTAKE_MOTOR_NAME = "INTAKE_MOTOR";
    private static final String INTAKE_SERVO_NAME = "INTAKE_SERVO";

    private static final double STOPPED_SPEED = 0.0;
    private static final double MOVING_SPEED = 1.0;
    private static final double INTAKING_SPEED = MOVING_SPEED;
    private static final double OUTTAKING_SPEED = MOVING_SPEED * -1;
    private static final double LOWERED_POSITION = 0.33;
    private static final double RAISED_POSITION = 0.0;

    public Intake() {
        initialize();
    }

    @Hardware(name = INTAKE_MOTOR_NAME)
    public DcMotor intakeMotor;

    @Hardware(name = INTAKE_SERVO_NAME)
    public Servo intakeServo;

    private IMotorState intakeMotorState;
    private IServoState intakeServoState;

    private void initialize() {
        intakeMotorState = new MotorState(INTAKE_MOTOR_NAME, false);
        intakeServoState = new ServoState(INTAKE_SERVO_NAME, Direction.FORWARD, LOWERED_POSITION);
    }

    @Override
    public String getName() {
        return this.getClass().getName();
    }

    @Override
    public List<? super State> getNextState() {
        return Arrays.asList(intakeMotorState, intakeServoState);
    }

    @Override
    public void raise() {
        intakeServoState = intakeServoState.withPosition(RAISED_POSITION);
    }

    @Override
    public void lower() {
        intakeServoState = intakeServoState.withPosition(LOWERED_POSITION);
    }

    @Override
    public void beginIntaking() {
        intakeMotorState = intakeMotorState.withPower(INTAKING_SPEED);
    }

    @Override
    public void beginOuttaking() {
        intakeMotorState = intakeMotorState.withPower(OUTTAKING_SPEED);
    }

    @Override
    public void stop() {
        intakeMotorState = intakeMotorState.withPower(STOPPED_SPEED);
    }

    @Override
    @Observable(key = "INTAKESTATE")
    public IntakeState getState() {
        if (intakeServoState.getPosition() == LOWERED_POSITION) {
            return intakeMotorState.getPower() == INTAKING_SPEED
                    ? IntakeState.LOWERED_AND_INTAKING : intakeMotorState.getPower() == OUTTAKING_SPEED
                    ? IntakeState.LOWERED_AND_OUTTAKING : IntakeState.LOWERED_AND_STOPPED;
        } else {
            return intakeMotorState.getPower() == INTAKING_SPEED
                    ? IntakeState.RAISED_AND_INTAKING : intakeMotorState.getPower() == OUTTAKING_SPEED
                    ? IntakeState.RAISED_AND_OUTTAKING : IntakeState.RAISED_AND_STOPPED;
        }
    }
}
