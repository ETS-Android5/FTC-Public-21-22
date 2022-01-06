package org.firstinspires.ftc.teamcode.hardware.robots.sliderliftbot;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.AutonomousOnly;
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

public class SliderLiftBot implements Component {
  public final IMecanumDrivetrain drivetrain = new MecanumDrivetrain();
  public final IRampedIntake intake = new RampedIntake();
  public final IFourHeightLift fourHeightLift = new FourHeightLift();
  public final IOuttakeBucket outtakeBucket = new OuttakeBucket();
  public final ICarouselSpinner carouselSpinner = new CarouselSpinner();

  @AutonomousOnly public final FtcCamera webcam = new Webcam();

  // Sensors store 5 data points at a time
  // Sample every 25 ms

  @SuppressWarnings("unused")
  @AutonomousOnly
  public final Interpolatable frontDistance =
      new InterpolatableRev2m(48, 25000000, 3, "CB_AUTO_FRONT_RANGE");

  @SuppressWarnings("unused")
  @AutonomousOnly
  public final Interpolatable leftDistance =
      new InterpolatableRev2m(48, 25000000, 3, "CB_AUTO_LEFT_RANGE");

  @SuppressWarnings("unused")
  @AutonomousOnly
  public final Interpolatable rightDistance =
      new InterpolatableRev2m(48, 25000000, 3, "CB_AUTO_RIGHT_RANGE");

  @SuppressWarnings("unused")
  @AutonomousOnly
  public final Interpolatable rearDistance =
      new InterpolatableRev2m(48, 25000000, 3, "CB_AUTO_REAR_RANGE");

  @SuppressWarnings("unused")
  @AutonomousOnly
  public final Interpolatable gyro = new InterpolatableRevGyro(48, 25000000, 3);

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
