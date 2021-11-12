package org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.annotations.Observable;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.MotorTrackerPipe;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.MotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Collections;
import java.util.List;

public class CarouselSpinner implements ICarouselSpinner {
    private static final String CAROUSEL_SPINNER_MOTOR_NAME = "CAROUSEL_SPINNER_MOTOR";

    private static final double CAROUSEL_CIRCUMFERENCE_IN = 15.0 * Math.PI;
    private static final double CAROUSEL_SPINNER_WHEEL_CIRCUMFERENCE_IN = 4.0 * Math.PI;
    private static final double MOTOR_TICKS_PER_MOTOR_REVOLUTION = 560.0;
    private static final double LOSS_FACTOR = 1.33;
    private static final int TICKS_PER_CAROUSEL_REVOLUTION =
            (int) (((CAROUSEL_CIRCUMFERENCE_IN / CAROUSEL_SPINNER_WHEEL_CIRCUMFERENCE_IN)
                    * MOTOR_TICKS_PER_MOTOR_REVOLUTION * LOSS_FACTOR) + .5);

    public CarouselSpinner() {
        initialize();
    }

    @Hardware(name = CAROUSEL_SPINNER_MOTOR_NAME, runMode = RunMode.RUN_TO_POSITION)
    public DcMotor spinnerMotor;

    private IMotorState spinnerMotorState;

    private CarouselSpinnerState carouselSpinnerState = CarouselSpinnerState.STOPPED;

    private void initialize() {
        spinnerMotorState = new MotorState(CAROUSEL_SPINNER_MOTOR_NAME, false)
                .withRunMode(RunMode.RUN_TO_POSITION)
                .withTargetPosition(0)
                .withPowerCurve(PowerCurves.CAROUSEL_CURVE);
    }

    @Override
    public String getName() {
        return this.getClass().getName();
    }

    @Override
    public List<? super State> getNextState() {
        return Collections.singletonList(spinnerMotorState.duplicate());
    }

    @Override
    public void spinForward() {
        spinnerMotorState = spinnerMotorState
                .withRunMode(RunMode.RUN_TO_POSITION)
                .withTargetPosition(MotorTrackerPipe.getInstance().getPositionOf(CAROUSEL_SPINNER_MOTOR_NAME) + TICKS_PER_CAROUSEL_REVOLUTION);
        carouselSpinnerState = CarouselSpinnerState.SPINNING_BACKWARD;
    }

    @Override
    public void spinBackward() {
        spinnerMotorState = spinnerMotorState
                .withRunMode(RunMode.RUN_TO_POSITION)
                .withTargetPosition(MotorTrackerPipe.getInstance().getPositionOf(CAROUSEL_SPINNER_MOTOR_NAME) - TICKS_PER_CAROUSEL_REVOLUTION);
        carouselSpinnerState = CarouselSpinnerState.SPINNING_FORWARD;
    }

    @Override
    @Observable(key = "CAROUSELSPINNER")
    public CarouselSpinnerState getState() {
        return carouselSpinnerState;
    }
}
