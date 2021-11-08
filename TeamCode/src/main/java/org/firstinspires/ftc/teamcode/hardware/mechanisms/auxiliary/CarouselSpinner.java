package org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.annotations.Observable;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.MotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Collections;
import java.util.List;

public class CarouselSpinner implements ICarouselSpinner {
    private static final String CAROUSEL_SPINNER_MOTOR_NAME = "CAROUSEL_SPINNER_MOTOR";

    private static final double STOPPED_SPEED = 0.0;
    private static final double MOVING_SPEED = 0.8;
    private static final double FORWARD_SPEED = MOVING_SPEED;
    private static final double BACKWARD_SPEED = MOVING_SPEED * -1;

    public CarouselSpinner() {
        initialize();
    }

    @Hardware(name = CAROUSEL_SPINNER_MOTOR_NAME)
    public DcMotor spinnerMotor;

    private IMotorState spinnerMotorState;

    private void initialize() {
        spinnerMotorState = new MotorState(CAROUSEL_SPINNER_MOTOR_NAME, false);
    }

    @Override
    public String getName() {
        return this.getClass().getName();
    }

    @Override
    public List<? super State> getNextState() {
        return Collections.singletonList(spinnerMotorState);
    }

    @Override
    public void spinForward() {
        spinnerMotorState = spinnerMotorState.withPower(FORWARD_SPEED);
    }

    @Override
    public void spinBackward() {
        spinnerMotorState = spinnerMotorState.withPower(BACKWARD_SPEED);
    }

    @Override
    public void stopSpinning() {
        spinnerMotorState = spinnerMotorState.withPower(STOPPED_SPEED);
    }

    @Override
    @Observable(key = "CAROUSELSPINNER")
    public CarouselSpinnerState getState() {
        return spinnerMotorState.getPower() == FORWARD_SPEED
                ? CarouselSpinnerState.SPINNING_FORWARD : spinnerMotorState.getPower() == BACKWARD_SPEED
                ? CarouselSpinnerState.SPINNING_BACKWARD : CarouselSpinnerState.STOPPED;
    }
}
