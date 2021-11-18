package org.firstinspires.ftc.teamcode.hardware.robots;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.AutonomousOnly;
import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;
import org.firstinspires.ftc.teamcode.hardware.detection.vision.FtcCamera;
import org.firstinspires.ftc.teamcode.hardware.detection.vision.Webcam;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.CarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.ICarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.IOuttakeBucket;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.OuttakeBucket;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains.IMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.intakes.IIntake;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.intakes.Intake;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts.FourHeightLift;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts.IFourHeightLift;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class MecanumBot implements Component {
  public final IMecanumDrivetrain drivetrain = new MecanumDrivetrain();
  public final IIntake intake = new Intake();
  public final IFourHeightLift fourHeightLift = new FourHeightLift();
  public final IOuttakeBucket outtakeBucket = new OuttakeBucket();
  public final ICarouselSpinner carouselSpinner = new CarouselSpinner();

  @AutonomousOnly public final FtcCamera webcam = new Webcam();

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
