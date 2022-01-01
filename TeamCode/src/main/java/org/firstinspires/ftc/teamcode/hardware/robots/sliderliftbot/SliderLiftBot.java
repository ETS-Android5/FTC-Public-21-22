package org.firstinspires.ftc.teamcode.hardware.robots.sliderliftbot;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.AutonomousOnly;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.fn.TriFunction;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.InterpolatablePipe;
import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.Interpolatable;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;
import org.firstinspires.ftc.teamcode.hardware.detection.distance.InterpolatableRev2m;
import org.firstinspires.ftc.teamcode.hardware.detection.gyro.InterpolatableRevGyro;
import org.firstinspires.ftc.teamcode.hardware.detection.vision.FtcCamera;
import org.firstinspires.ftc.teamcode.hardware.detection.vision.Webcam;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.CarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.ICarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.IOuttakeBucket;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.OuttakeBucket;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains.IMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.intakes.IRampedIntake;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.intakes.RampedIntake;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts.FourHeightLift;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts.IFourHeightLift;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

public class SliderLiftBot implements Component {
  public final IMecanumDrivetrain drivetrain = new MecanumDrivetrain();
  public final IRampedIntake intake = new RampedIntake();
  public final IFourHeightLift fourHeightLift = new FourHeightLift();
  public final IOuttakeBucket outtakeBucket = new OuttakeBucket();
  public final ICarouselSpinner carouselSpinner = new CarouselSpinner();

  @AutonomousOnly public final FtcCamera webcam = new Webcam();

  // Sensors store 5 data points at a time
  // Sample every 25 ms

  @AutonomousOnly
  public final Interpolatable frontDistance =
      new InterpolatableRev2m(48, 25000000, 3, "CB_AUTO_FRONT_RANGE");

  @AutonomousOnly
  public final Interpolatable leftDistance =
      new InterpolatableRev2m(48, 25000000, 3, "CB_AUTO_LEFT_RANGE");

  @AutonomousOnly
  public final Interpolatable rightDistance =
      new InterpolatableRev2m(48, 25000000, 3, "CB_AUTO_RIGHT_RANGE");

  @AutonomousOnly
  public final Interpolatable rearDistance =
      new InterpolatableRev2m(48, 25000000, 3, "CB_AUTO_REAR_RANGE");

  @AutonomousOnly public final Interpolatable gyro = new InterpolatableRevGyro(48, 25000000, 3);

  public void runAtHeadingUntilCondition(
      int target,
      int tolerance,
      Direction direction,
      Supplier<Boolean> condition,
      Supplier<Double> basePower,
      Supplier<Boolean> opModeIsActive,
      Runnable update) {
    TriFunction<Integer, Integer, Integer, Double> powerCurve =
        PowerCurves.generatePowerCurve(0.25, 2.33);
    Double currentHeading = InterpolatablePipe.getInstance().currentDataPointOf("CB_AUTO_IMU");
    double initialHeading = currentHeading;
    Double turnSpeed;
    int leftDirection;
    int rightDirection;
    if (Math.abs(target - currentHeading) > tolerance) {
      while (opModeIsActive.get() && Math.abs(target - currentHeading) > tolerance) {
        currentHeading = InterpolatablePipe.getInstance().currentDataPointOf("CB_AUTO_IMU");
        turnSpeed =
            powerCurve.apply(
                (int) Math.round(currentHeading), (int) Math.round(initialHeading), target);
        leftDirection = currentHeading > target ? -1 : 1;
        rightDirection = currentHeading < target ? -1 : 1;
        drivetrain.setLeftPower(turnSpeed * leftDirection);
        drivetrain.setRightPower(turnSpeed * rightDirection);
        update.run();
      }
    }
    drivetrain.setAllPower(0);
    update.run();
    double leftDiff;
    double rightDiff;
    double directionAdjustment = direction == Direction.FORWARD ? -1 : 1;
    while (opModeIsActive.get() && !condition.get()) {
      currentHeading = InterpolatablePipe.getInstance().currentDataPointOf("CB_AUTO_IMU");
      turnSpeed =
          powerCurve.apply(
              (int) Math.round(currentHeading), (int) Math.round(initialHeading), target);
      leftDiff = turnSpeed * (currentHeading > target ? -1 : 1) * directionAdjustment;
      rightDiff = turnSpeed * (currentHeading < target ? -1 : 1) * directionAdjustment;
      double basePowerVal = basePower.get();
      drivetrain.setLeftPower((basePowerVal * directionAdjustment) + leftDiff);
      drivetrain.setRightPower((basePowerVal * directionAdjustment) + rightDiff);
      update.run();
    }
    drivetrain.setAllPower(0);
    update.run();
  }

  @Override
  public String getName() {
    return this.getClass().getName();
  }

  @Override
  public List<? super State> getNextState() {
    List<? super State> states = new LinkedList<>();
    Arrays.asList(drivetrain, intake, fourHeightLift, outtakeBucket, carouselSpinner)
        .forEach((component) -> component.getNextState().forEach((s) -> states.add((State) s)));
    return states;
  }
}
