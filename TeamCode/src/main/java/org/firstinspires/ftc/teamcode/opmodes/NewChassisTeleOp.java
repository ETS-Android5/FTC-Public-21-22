package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.controller.BooleanSurface;
import org.firstinspires.ftc.teamcode.core.controller.ScalarSurface;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedTeleOp;
import org.firstinspires.ftc.teamcode.hardware.robots.NewChassis;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(name = "NewChassis")
public class NewChassisTeleOp extends EnhancedTeleOp {
  private static final double MAX_TURRET_ADJUSTMENT = 12; // 12 ticks or 15 degrees
  private static final double MAX_FIRST_JOINT_ADJUSTMENT = 50; // 50 ticks
  private static final double MAX_SECOND_JOINT_ADJUSTMENT = 0.055; // 15 degrees
  private final NewChassis robot;
  private final AtomicBoolean halfSpeed = new AtomicBoolean(false);
  private final AtomicBoolean alreadyIntaking = new AtomicBoolean(false);
  private final AtomicBoolean alreadyOuttaking = new AtomicBoolean(false);
  private final ScheduledExecutorService executorService =
      Executors.newSingleThreadScheduledExecutor();

  public NewChassisTeleOp() {
    super(new NewChassis());
    this.robot = (NewChassis) super.robotObject;
  }

  private static double THIRD_MANIPULATION(double in) {
    return Math.pow(in, 3);
  }

  @Override
  public void onInitPressed() {}

  @Override
  public void initLoop() {}

  @Override
  public void onStartPressed() {
    controller1.setManipulation(NewChassisTeleOp::THIRD_MANIPULATION, ScalarSurface.LEFT_STICK_Y);
    controller1.registerOnPressedCallback(
        () -> halfSpeed.set(!halfSpeed.get()), true, BooleanSurface.X);
    controller2.registerOnPressedCallback(
        () -> {
          if (alreadyIntaking.get()) {
            robot.intake.stop();
            alreadyIntaking.set(false);
          } else {
            robot.intake.beginIntaking();
            alreadyIntaking.set(true);
          }
        },
        true,
        BooleanSurface.RIGHT_BUMPER);
    controller2.registerOnPressedCallback(
        () -> {
          if (alreadyOuttaking.get()) {
            robot.intake.stop();
            alreadyOuttaking.set(false);
          } else {
            robot.intake.beginOuttaking();
            alreadyOuttaking.set(true);
          }
        },
        true,
        BooleanSurface.LEFT_BUMPER);
    controller2.registerOnPressedCallback(robot.gripper::toggle, true, BooleanSurface.DPAD_RIGHT);
    controller2.registerOnPressedCallback(
        () -> {
          // intake position
          robot.gripper.close();
          robot.lift.setArmOnePosition(300); // TODO: Trim to proper height
          robot.lift.setArmTwoPosition(
              0.33); // 90 degrees to the arm, ideally, on an ideal range of 0 to 0.66
          executorService.schedule(
              () -> {
                robot.turret.turnToFront();
                robot.lift.setArmOnePosition(0);
                executorService.schedule(robot.gripper::open, 1000, TimeUnit.MILLISECONDS);
              },
              1000,
              TimeUnit.MILLISECONDS);
        },
        true,
        BooleanSurface.A);
    controller2.registerOnPressedCallback(
        () -> {
          // right angle
          robot.gripper.close();
          robot.lift.setArmOnePosition(300); // TODO: Trim to proper height
          robot.lift.setArmTwoPosition(0.3); // Tucked in. See range above
          executorService.schedule(robot.turret::turnToRight, 1000, TimeUnit.MILLISECONDS);
        },
        true,
        BooleanSurface.B);
    controller2.registerOnPressedCallback(
        () -> {
          // top alliance hub tier
          robot.gripper.close();
          robot.lift.setArmOnePosition(800); // TODO: Trim to proper height
          robot.lift.setArmTwoPosition(0.3); // Tucked in. See range above
          executorService.schedule(robot.turret::turnCWToBack, 1000, TimeUnit.MILLISECONDS);
        },
        true,
        BooleanSurface.X);
    controller2.registerOnPressedCallback(
        () -> {
          // capping position
          robot.gripper.close();
          robot.lift.setArmOnePosition(1000); // TODO: Trim to proper height
          robot.lift.setArmTwoPosition(0.3); // Tucked in. See range above
          executorService.schedule(robot.turret::turnCWToBack, 1000, TimeUnit.MILLISECONDS);
        },
        true,
        BooleanSurface.Y);
    controller2.registerOnPressedCallback(
        robot.carouselSpinner::spinForward, true, BooleanSurface.LEFT_STICK);
    controller2.registerOnPressedCallback(
        robot.carouselSpinner::spinBackward, true, BooleanSurface.RIGHT_STICK);
  }

  @Override
  public void onLoop() {
    double turnValue = controller1.rightStickX();
    double speed = halfSpeed.get() ? 0.5 : 1;
    robot.drivetrain.driveBySticks(
        controller1.leftStickX() * speed, controller1.leftStickY() * speed, turnValue * speed);
    robot.turret.turnToDegrees(
        (int)
            Math.round(
                robot.turret.getState()
                    + (controller2.rightTrigger() * MAX_TURRET_ADJUSTMENT)
                    - (controller2.leftTrigger() * MAX_TURRET_ADJUSTMENT)));
    robot.lift.setArmOnePosition(
        (int)
            Math.round(
                robot.lift.getState().first
                    + (controller2.leftStickY() * MAX_FIRST_JOINT_ADJUSTMENT)));
    robot.lift.setArmTwoPosition(
        (int)
            Math.round(
                robot.lift.getState().second
                    + (controller2.rightStickY() * MAX_SECOND_JOINT_ADJUSTMENT)));
  }

  @Override
  public void onStop() {}
}
