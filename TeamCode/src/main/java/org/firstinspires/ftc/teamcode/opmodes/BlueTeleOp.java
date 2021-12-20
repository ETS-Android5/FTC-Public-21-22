package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.controller.BooleanSurface;
import org.firstinspires.ftc.teamcode.core.controller.ScalarSurface;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedTeleOp;
import org.firstinspires.ftc.teamcode.hardware.robots.NewChassis;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(name = "Blue")
@Disabled
public class BlueTeleOp extends EnhancedTeleOp {
  private static final double MAX_TURRET_ADJUSTMENT = 12; // 12 ticks or 15 degrees
  private static final double MAX_FIRST_JOINT_ADJUSTMENT = 50; // 50 ticks
  private static final double MAX_SECOND_JOINT_ADJUSTMENT = 0.055; // 15 degrees

  private final NewChassis robot;

  private final AtomicBoolean halfSpeed = new AtomicBoolean(false);

  private final AtomicBoolean alreadyIntaking = new AtomicBoolean(false);
  private final AtomicBoolean alreadyOuttaking = new AtomicBoolean(false);

  private final AtomicBoolean allianceHubMode = new AtomicBoolean(false);
  private final AtomicBoolean tippedMode = new AtomicBoolean(false);

  private final ScheduledExecutorService executorService =
      Executors.newSingleThreadScheduledExecutor();

  public BlueTeleOp() {
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
    controller1.setManipulation(BlueTeleOp::THIRD_MANIPULATION, ScalarSurface.LEFT_STICK_Y);
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
          robot.lift.setArmOnePosition(0);
          robot.lift.setArmTwoPosition(0.47);
          executorService.schedule(
              () -> {
                robot.turret.turnToFront();
                robot.lift.setArmOnePosition(-140);
                executorService.schedule(robot.gripper::open, 1000, TimeUnit.MILLISECONDS);
              },
              1000,
              TimeUnit.MILLISECONDS);
        },
        true,
        BooleanSurface.A);
    controller2.registerOnPressedCallback(
        robot.carouselSpinner::spinForward, true, BooleanSurface.LEFT_STICK);
    controller2.registerOnPressedCallback(
        () -> {
          boolean allianceHubModeTmp = allianceHubMode.get();
          allianceHubMode.set(!allianceHubModeTmp);
        },
        true,
        BooleanSurface.BACK);

    // Mode specific controls
    controller2.registerOnPressedCallback(
        () -> {
          if (allianceHubMode.get()) {
            robot.gripper.close();
            robot.lift.setArmOnePosition(0);
            robot.lift.setArmTwoPosition(0.47);
            executorService.schedule(
                () -> {
                  robot.turret.turnCWToBack();
                  executorService.schedule(
                      () -> {
                        robot.lift.setArmOnePosition(-370);
                        robot.lift.setArmTwoPosition(0);
                      },
                      1000,
                      TimeUnit.MILLISECONDS);
                },
                1000,
                TimeUnit.MILLISECONDS);
          } else {
            if (tippedMode.get()) {
              robot.gripper.close();
              // TODO: Tipped mode values
            } else {
              robot.gripper.close();
              robot.lift.setArmOnePosition(69);
              robot.lift.setArmTwoPosition(0.47); // Tucked in. See range above
              executorService.schedule(
                  () -> {
                    robot.turret.turnToLeft();
                    executorService.schedule(
                        () -> robot.lift.setArmTwoPosition(0.56), 1000, TimeUnit.MILLISECONDS);
                  },
                  1000,
                  TimeUnit.MILLISECONDS);
            }
          }
        },
        true,
        BooleanSurface.B);
    controller2.registerOnPressedCallback(
        () -> {
          if (allianceHubMode.get()) {
            robot.gripper.close();
            robot.lift.setArmOnePosition(0);
            robot.lift.setArmTwoPosition(0.47);
            executorService.schedule(
                () -> {
                  robot.turret.turnCWToBack();
                  executorService.schedule(
                      () -> {
                        robot.lift.setArmOnePosition(-18);
                        robot.lift.setArmTwoPosition(0.13);
                      },
                      1000,
                      TimeUnit.MILLISECONDS);
                },
                1000,
                TimeUnit.MILLISECONDS);
          } else {
            if (tippedMode.get()) {
              robot.gripper.close();
              // TODO: Tipped mode values
            } else {
              robot.gripper.close();
              robot.lift.setArmOnePosition(0);
              robot.lift.setArmTwoPosition(0.47);
              executorService.schedule(
                  () -> {
                    robot.turret.turnToLeft();
                    executorService.schedule(
                        () -> {
                          robot.lift.setArmOnePosition(-230);
                          robot.lift.setArmTwoPosition(0.13);
                        },
                        1000,
                        TimeUnit.MILLISECONDS);
                  },
                  1000,
                  TimeUnit.MILLISECONDS);
            }
          }
        },
        true,
        BooleanSurface.X);
    controller2.registerOnPressedCallback(
        () -> {
          if (allianceHubMode.get()) {
            robot.gripper.close();
            robot.lift.setArmTwoPosition(570);
            robot.lift.setArmTwoPosition(0.47);
            executorService.schedule(
                () -> {
                  robot.turret.turnCWToBack();
                  executorService.schedule(
                      () -> robot.lift.setArmTwoPosition(0.43), 1000, TimeUnit.MILLISECONDS);
                },
                1000,
                TimeUnit.MILLISECONDS);
          } else {
            if (tippedMode.get()) {
              robot.gripper.close();
              // TODO: Tipped mode values
            } else {
              robot.gripper.close();
              robot.lift.setArmOnePosition(10);
              robot.lift.setArmTwoPosition(0.47);
              executorService.schedule(
                  () -> {
                    robot.turret.turnToLeft();
                    executorService.schedule(
                        () -> robot.lift.setArmTwoPosition(0.39), 1000, TimeUnit.MILLISECONDS);
                  },
                  1000,
                  TimeUnit.MILLISECONDS);
            }
          }
        },
        true,
        BooleanSurface.Y);
    controller2.registerOnPressedCallback(
        () -> {
          if (allianceHubMode.get()) {
            robot.gripper.close();
            robot.lift.setArmOnePosition(0);
            robot.lift.setArmTwoPosition(0.47);
            executorService.schedule(
                () -> {
                  robot.turret.turnToRight();
                  executorService.schedule(
                      () -> {
                        robot.lift.setArmOnePosition(-520);
                        robot.lift.setArmTwoPosition(0);
                        robot.gripper.open();
                      },
                      1000,
                      TimeUnit.MILLISECONDS);
                },
                1000,
                TimeUnit.MILLISECONDS);
          }
        },
        true,
        BooleanSurface.DPAD_LEFT);
    controller2.registerOnPressedCallback(
        () -> {
          if (allianceHubMode.get()) {
            robot.gripper.grabTeamMarker();
            executorService.schedule(
                () -> {
                  robot.lift.setArmOnePosition(570);
                  robot.lift.setArmTwoPosition(0.3);
                  robot.turret.turnCWToBack();
                },
                1000,
                TimeUnit.MILLISECONDS);
          } else {
            boolean tippedModeTmp = tippedMode.get();
            tippedMode.set(!tippedModeTmp);
          }
        },
        true,
        BooleanSurface.DPAD_UP);
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
