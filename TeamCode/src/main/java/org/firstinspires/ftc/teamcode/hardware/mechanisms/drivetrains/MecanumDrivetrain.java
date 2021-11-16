package org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.fn.TriFunction;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.CallbackData;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.MotorTrackerPipe;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.MotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

public class MecanumDrivetrain implements IMecanumDrivetrain {
    private static final String FRONT_LEFT_MOTOR_NAME = "FRONT_LEFT_MOTOR";
    private static final String FRONT_RIGHT_MOTOR_NAME = "FRONT_RIGHT_MOTOR";
    private static final String REAR_LEFT_MOTOR_NAME = "REAR_LEFT_MOTOR";
    private static final String REAR_RIGHT_MOTOR_NAME = "REAR_RIGHT_MOTOR";

    public MecanumDrivetrain() {
        initialize();
    }

    @Hardware(name = FRONT_LEFT_MOTOR_NAME)
    public DcMotor frontLeftMotor;
    @Hardware(name = FRONT_RIGHT_MOTOR_NAME, direction = Direction.REVERSE)
    public DcMotor frontRightMotor;
    @Hardware(name = REAR_LEFT_MOTOR_NAME)
    public DcMotor rearLeftMotor;
    @Hardware(name = REAR_RIGHT_MOTOR_NAME, direction = Direction.REVERSE)
    public DcMotor rearRightMotor;

    protected IMotorState frontLeftMotorState = new MotorState(FRONT_LEFT_MOTOR_NAME, true);
    protected IMotorState frontRightMotorState = new MotorState(FRONT_RIGHT_MOTOR_NAME, false);
    protected IMotorState rearLeftMotorState = new MotorState(REAR_LEFT_MOTOR_NAME, true);
    protected IMotorState rearRightMotorState = new MotorState(REAR_RIGHT_MOTOR_NAME, false);

    private void initialize() {
        frontLeftMotorState = new MotorState(FRONT_LEFT_MOTOR_NAME, true);
        frontRightMotorState = new MotorState(FRONT_RIGHT_MOTOR_NAME, false);
        rearLeftMotorState = new MotorState(REAR_LEFT_MOTOR_NAME, true);
        rearRightMotorState = new MotorState(REAR_RIGHT_MOTOR_NAME, false);
    }

    @Override
    public void setAllPower(double power) {
        frontLeftMotorState = frontLeftMotorState.withPower(power);
        frontRightMotorState = frontRightMotorState.withPower(power);
        rearLeftMotorState = rearLeftMotorState.withPower(power);
        rearRightMotorState = rearRightMotorState.withPower(power);
    }

    @Override
    public void setLeftPower(double power) {
        frontLeftMotorState = frontLeftMotorState.withPower(power);
        rearLeftMotorState = rearLeftMotorState.withPower(power);
    }

    @Override
    public void setRightPower(double power) {
        frontRightMotorState = frontRightMotorState.withPower(power);
        rearRightMotorState = rearRightMotorState.withPower(power);
    }

    @Override
    public void setAllTarget(int ticks) {
        frontLeftMotorState = frontLeftMotorState.withTargetPosition(ticks);
        frontRightMotorState = frontRightMotorState.withTargetPosition(ticks);
        rearLeftMotorState = rearLeftMotorState.withTargetPosition(ticks);
        rearRightMotorState = rearRightMotorState.withTargetPosition(ticks);
    }

    @Override
    public void setLeftTarget(int ticks) {
        frontLeftMotorState = frontLeftMotorState.withTargetPosition(ticks);
        rearLeftMotorState = rearLeftMotorState.withTargetPosition(ticks);
    }

    @Override
    public void setRightTarget(int ticks) {
        frontRightMotorState = frontRightMotorState.withTargetPosition(ticks);
        rearRightMotorState = rearRightMotorState.withTargetPosition(ticks);
    }

    @Override
    public void reset() {
        initialize();
    }

    @Override
    public void setRunMode(RunMode runMode) {
        frontLeftMotorState = frontLeftMotorState.withRunMode(runMode);
        frontRightMotorState = frontRightMotorState.withRunMode(runMode);
        rearLeftMotorState = rearLeftMotorState.withRunMode(runMode);
        rearRightMotorState = rearRightMotorState.withRunMode(runMode);
    }

    @Override
    public String getName() {
        return this.getClass().getName();
    }

    @Override
    public List<? super State> getNextState() {
        return Arrays.asList(
                frontLeftMotorState,
                frontRightMotorState,
                rearLeftMotorState,
                rearRightMotorState
        );
    }

    @Override
    public void setFrontLeftPower(double power) {
        frontLeftMotorState = frontLeftMotorState.withPower(power);
    }

    @Override
    public void setFrontRightPower(double power) {
        frontRightMotorState = frontRightMotorState.withPower(power);
    }

    @Override
    public void setRearLeftPower(double power) {
        rearLeftMotorState = rearLeftMotorState.withPower(power);
    }

    @Override
    public void setRearRightPower(double power) {
        rearRightMotorState = rearRightMotorState.withPower(power);
    }

    @Override
    public void setFrontLeftTarget(int ticks) {
        frontLeftMotorState = frontLeftMotorState.withTargetPosition(ticks);
    }

    @Override
    public void setFrontRightTarget(int ticks) {
        frontRightMotorState = frontRightMotorState.withTargetPosition(ticks);
    }

    @Override
    public void setRearLeftTarget(int ticks) {
        rearLeftMotorState = rearLeftMotorState.withTargetPosition(ticks);
    }

    @Override
    public void setRearRightTarget(int ticks) {
        rearRightMotorState = rearRightMotorState.withTargetPosition(ticks);
    }

    @Override
    public void driveBySticks(double lateral, double longitudinal, double turn) {
        double rearWheelPower = Math.hypot(lateral, longitudinal);
        double rearStickAngleRadians = Math.atan2(longitudinal, lateral) - Math.PI / 4;

        double rearSinAngleRadians = Math.sin(rearStickAngleRadians);
        double rearCosAngleRadians = Math.cos(rearStickAngleRadians);

        double frontWheelPower = Math.hypot(lateral, -longitudinal);
        double frontStickAngleRadians = Math.atan2(-longitudinal, lateral) - Math.PI / 4;

        double frontSinAngleRadians = Math.sin(frontStickAngleRadians);
        double frontCosAngleRadians = Math.cos(frontStickAngleRadians);

        
        double rearFactor = 1 / Math.max(Math.abs(rearSinAngleRadians), Math.abs(rearCosAngleRadians));

        double frontFactor = 1 / Math.max(Math.abs(frontSinAngleRadians), Math.abs(frontCosAngleRadians));

        frontLeftMotorState = frontLeftMotorState.withPower(frontWheelPower * frontCosAngleRadians * frontFactor + turn);
        frontRightMotorState = frontRightMotorState.withPower(frontWheelPower * frontSinAngleRadians * frontFactor - turn);
        rearLeftMotorState = rearLeftMotorState.withPower(rearWheelPower * rearSinAngleRadians * rearFactor + turn);
        rearRightMotorState = rearRightMotorState.withPower(rearWheelPower * rearCosAngleRadians * rearFactor - turn);
    }

    @Override
    public void setPowerCurve(TriFunction<Integer, Integer, Integer, Double> powerCurve) {
        frontLeftMotorState = frontLeftMotorState.withPowerCurve(powerCurve);
        frontRightMotorState = frontRightMotorState.withPowerCurve(powerCurve);
        rearLeftMotorState = rearLeftMotorState.withPowerCurve(powerCurve);
        rearRightMotorState = rearRightMotorState.withPowerCurve(powerCurve);
    }

    @Override
    public void autoRunToPosition(int target, int toleranceTicks, Supplier<Boolean> opModeIsActive, Runnable update) {
        AtomicBoolean reachedFrontLeftTarget = new AtomicBoolean(false);
        AtomicBoolean reachedFrontRightTarget = new AtomicBoolean(false);
        AtomicBoolean reachedRearLeftTarget = new AtomicBoolean(false);
        AtomicBoolean reachedRearRightTarget = new AtomicBoolean(false);
        setAllTarget(target);
        MotorTrackerPipe.getInstance().setCallbackForMotorPosition(new CallbackData(
                getFrontLeftName(),
                target,
                toleranceTicks,
                () -> reachedFrontLeftTarget.set(true)
        ));
        MotorTrackerPipe.getInstance().setCallbackForMotorPosition(new CallbackData(
                getFrontRightName(),
                target,
                toleranceTicks,
                () -> reachedFrontRightTarget.set(true)
        ));
        MotorTrackerPipe.getInstance().setCallbackForMotorPosition(new CallbackData(
                getRearLeftName(),
                target,
                toleranceTicks,
                () -> reachedRearLeftTarget.set(true)
        ));
        MotorTrackerPipe.getInstance().setCallbackForMotorPosition(new CallbackData(
                getRearRightName(),
                target,
                toleranceTicks,
                () -> reachedRearRightTarget.set(true)
        ));
        while (opModeIsActive.get()
                && !reachedFrontLeftTarget.get()
                && !reachedFrontRightTarget.get()
                && !reachedRearLeftTarget.get()
                && !reachedRearRightTarget.get()) {
            update.run();
        }
    }

    @Override
    public String getFrontLeftName() {
        return frontLeftMotorState.getName();
    }

    @Override
    public String getFrontRightName() {
        return frontRightMotorState.getName();
    }

    @Override
    public String getRearLeftName() {
        return rearLeftMotorState.getName();
    }

    @Override
    public String getRearRightName() {
        return rearRightMotorState.getName();
    }
}
