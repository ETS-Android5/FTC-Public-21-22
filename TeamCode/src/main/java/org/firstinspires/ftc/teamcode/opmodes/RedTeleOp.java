package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.controller.BooleanSurface;
import org.firstinspires.ftc.teamcode.core.controller.ScalarSurface;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.CallbackData;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.ExitPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.MotorTrackerPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.StateFilterResult;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedTeleOp;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.Turret;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts.DualJointAngularLift;
import org.firstinspires.ftc.teamcode.hardware.robots.NewChassis;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(name = "Red")
public class RedTeleOp extends EnhancedTeleOp {
  private static final double MAX_TURRET_ADJUSTMENT = 1; // 1 tick or 1.25 degrees
  private static final double MAX_FIRST_JOINT_ADJUSTMENT = 2.5; // ticks
  private static final double MAX_SECOND_JOINT_ADJUSTMENT = 0.0055; // 1.5 degrees

  private static final int firstJointOffset = 230;

  private final NewChassis robot;

  private final AtomicBoolean halfSpeed = new AtomicBoolean(false);

  private final AtomicBoolean alreadyIntaking = new AtomicBoolean(false);
  private final AtomicBoolean alreadyOuttaking = new AtomicBoolean(false);

  private final AtomicBoolean allianceHubMode = new AtomicBoolean(false);
  private final AtomicBoolean tippedMode = new AtomicBoolean(false);

  private final ScheduledExecutorService executorService =
      Executors.newSingleThreadScheduledExecutor();

  private boolean previouslyTrimming = false;

  public RedTeleOp() {
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
    controller1.setManipulation(RedTeleOp::THIRD_MANIPULATION, ScalarSurface.LEFT_STICK_Y);
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
        BooleanSurface.LEFT_BUMPER);
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
        BooleanSurface.RIGHT_BUMPER);
    controller2.registerOnPressedCallback(robot.gripper::toggle, true, BooleanSurface.DPAD_RIGHT);
    controller2.registerOnPressedCallback(
        () -> {
          // intake position
          robot.gripper.close();
          executorService.schedule(
              () ->
                  ExitPipe.getInstance()
                      .onNextTick(
                          () -> {
                            double currentTurretPos = robot.turret.getState();
                            double target = Turret.DEGREES_RIGHT * Turret.DEGREES_TO_TICKS;
                            if (Math.abs(currentTurretPos - target) < 5) {
                              robot.lift.setArmOnePosition(0);
                              robot.lift.setArmTwoPosition(0.47);
                              robot.turret.turnToFront();
                            } else {
                              robot.lift.setArmOnePosition(firstJointOffset);
                              robot.lift.setArmTwoPosition(0.47);
                              MotorTrackerPipe.getInstance()
                                  .setCallbackForMotorPosition(
                                      new CallbackData<>(
                                          DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME,
                                          (Integer ticks) -> ticks < 5 && ticks > -5,
                                          () -> {
                                            robot.turret.turnToFront();
                                            ExitPipe.getInstance()
                                                .onNextTick(
                                                    () ->
                                                        MotorTrackerPipe.getInstance()
                                                            .setCallbackForMotorPosition(
                                                                new CallbackData<>(
                                                                    Turret.TURRET_MOTOR_NAME,
                                                                    (Integer ticks) ->
                                                                        ticks < 5 && ticks > -5,
                                                                    () -> {
                                                                      robot.lift.setArmOnePosition(
                                                                          firstJointOffset + -160);
                                                                      robot.lift.setArmTwoPosition(
                                                                          0.47);
                                                                      robot.gripper.open();
                                                                    })));
                                          }));
                            }
                          }),
              100,
              TimeUnit.MILLISECONDS);
        },
        true,
        BooleanSurface.A);
    controller2.registerOnPressedCallback(
        robot.carouselSpinner::spinBackward, true, BooleanSurface.LEFT_STICK);
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
          robot.gripper.close();
          robot.intake.beginIntaking(); // Actually outtaking
          executorService.schedule(
              () -> ExitPipe.getInstance().onNextTick(robot.intake::stop),
              750,
              TimeUnit.MILLISECONDS);
          executorService.schedule(
              () ->
                  ExitPipe.getInstance()
                      .onNextTick(
                          () -> {
                            if (allianceHubMode.get()) {
                              double currentTurretPos = robot.turret.getState();
                              double target = -Turret.DEGREES_BACK * Turret.DEGREES_TO_TICKS;
                              if (Math.abs(currentTurretPos - target) < 5) {
                                robot.lift.setArmOnePosition(firstJointOffset + -390);
                                robot.lift.setArmTwoPosition(0);
                                robot.turret.turnCCWToBack();
                              } else {
                                robot.lift.setArmOnePosition(firstJointOffset);
                                robot.lift.setArmTwoPosition(0.47);
                                MotorTrackerPipe.getInstance()
                                    .setCallbackForMotorPosition(
                                        new CallbackData<>(
                                            DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME,
                                            (Integer ticks) -> ticks > -5 && ticks < 5,
                                            () -> {
                                              robot.turret.turnCCWToBack();
                                              ExitPipe.getInstance()
                                                  .onNextTick(
                                                      () ->
                                                          MotorTrackerPipe.getInstance()
                                                              .setCallbackForMotorPosition(
                                                                  new CallbackData<>(
                                                                      Turret.TURRET_MOTOR_NAME,
                                                                      (Integer ticks) ->
                                                                          ticks > (target - 5)
                                                                              && ticks
                                                                                  < (target + 5),
                                                                      () -> {
                                                                        robot.lift
                                                                            .setArmOnePosition(
                                                                                firstJointOffset
                                                                                    + -390);
                                                                        robot.lift
                                                                            .setArmTwoPosition(0);
                                                                      })));
                                            }));
                              }
                            } else {
                              if (tippedMode.get()) {
                                // TODO: Tipped mode values
                              } else {
                                double currentTurretPos = robot.turret.getState();
                                double target = Turret.DEGREES_RIGHT * Turret.DEGREES_TO_TICKS;
                                if (Math.abs(currentTurretPos - target) < 5) {
                                  robot.lift.setArmOnePosition(firstJointOffset + 69);
                                  robot.lift.setArmTwoPosition(0.56);
                                  robot.turret.turnToRight();
                                } else {
                                  robot.lift.setArmOnePosition(firstJointOffset + 69);
                                  robot.lift.setArmTwoPosition(0.47);
                                  MotorTrackerPipe.getInstance()
                                      .setCallbackForMotorPosition(
                                          new CallbackData<>(
                                              DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME,
                                              (Integer ticks) ->
                                                  ticks > (69 - 5) && ticks < (69 + 5),
                                              () -> {
                                                robot.turret.turnToRight();
                                                ExitPipe.getInstance()
                                                    .onNextTick(
                                                        () ->
                                                            MotorTrackerPipe.getInstance()
                                                                .setCallbackForMotorPosition(
                                                                    new CallbackData<>(
                                                                        Turret.TURRET_MOTOR_NAME,
                                                                        (Integer ticks) ->
                                                                            ticks > (target - 5)
                                                                                && ticks
                                                                                    < (target + 5),
                                                                        () ->
                                                                            robot.lift
                                                                                .setArmTwoPosition(
                                                                                    0.56))));
                                              }));
                                }
                              }
                            }
                          }),
              100,
              TimeUnit.MILLISECONDS);
        },
        true,
        BooleanSurface.B);
    controller2.registerOnPressedCallback(
        () -> {
          robot.gripper.close();
          robot.intake.beginIntaking(); // Actually outtaking
          executorService.schedule(
              () -> ExitPipe.getInstance().onNextTick(robot.intake::stop),
              750,
              TimeUnit.MILLISECONDS);
          executorService.schedule(
              () ->
                  ExitPipe.getInstance()
                      .onNextTick(
                          () -> {
                            if (allianceHubMode.get()) {
                              double currentTurretPos = robot.turret.getState();
                              double target = -Turret.DEGREES_BACK * Turret.DEGREES_TO_TICKS;
                              if (Math.abs(currentTurretPos - target) < 5) {
                                robot.lift.setArmOnePosition(firstJointOffset + -22);
                                robot.lift.setArmTwoPosition(0.13);
                                robot.turret.turnCCWToBack();
                              } else {
                                robot.lift.setArmOnePosition(firstJointOffset);
                                robot.lift.setArmTwoPosition(0.47);
                                MotorTrackerPipe.getInstance()
                                    .setCallbackForMotorPosition(
                                        new CallbackData<>(
                                            DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME,
                                            (Integer ticks) -> ticks > -5 && ticks < 5,
                                            () -> {
                                              robot.turret.turnCCWToBack();
                                              ExitPipe.getInstance()
                                                  .onNextTick(
                                                      () ->
                                                          MotorTrackerPipe.getInstance()
                                                              .setCallbackForMotorPosition(
                                                                  new CallbackData<>(
                                                                      Turret.TURRET_MOTOR_NAME,
                                                                      (Integer ticks) ->
                                                                          ticks > target - 5
                                                                              && ticks < target + 5,
                                                                      () -> {
                                                                        robot.lift
                                                                            .setArmOnePosition(
                                                                                firstJointOffset
                                                                                    + -22);
                                                                        robot.lift
                                                                            .setArmTwoPosition(
                                                                                0.13);
                                                                      })));
                                            }));
                              }
                            } else {
                              if (tippedMode.get()) {
                                // TODO: Tipped mode values
                              } else {
                                double currentTurretPos = robot.turret.getState();
                                double target = Turret.DEGREES_RIGHT * Turret.DEGREES_TO_TICKS;
                                if (Math.abs(currentTurretPos - target) < 5) {
                                  robot.lift.setArmOnePosition(firstJointOffset + 69);
                                  robot.turret.turnToRight();
                                  MotorTrackerPipe.getInstance()
                                      .setCallbackForMotorPosition(
                                          new CallbackData<>(
                                              DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME,
                                              (Integer ticks) ->
                                                  ticks > (69 - 5) && ticks < (69 + 5),
                                              () -> {
                                                robot.lift.setArmTwoPosition(0.13);
                                                executorService.schedule(
                                                    () ->
                                                        ExitPipe.getInstance()
                                                            .onNextTick(
                                                                () ->
                                                                    robot.lift.setArmOnePosition(
                                                                        firstJointOffset + -230)),
                                                    100,
                                                    TimeUnit.MILLISECONDS);
                                              }));
                                } else {
                                  robot.lift.setArmOnePosition(firstJointOffset + 69);
                                  robot.lift.setArmTwoPosition(0.47);
                                  MotorTrackerPipe.getInstance()
                                      .setCallbackForMotorPosition(
                                          new CallbackData<>(
                                              DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME,
                                              (Integer ticks) ->
                                                  ticks > (69 - 5) && ticks < (69 + 5),
                                              () -> {
                                                robot.turret.turnToRight();
                                                ExitPipe.getInstance()
                                                    .onNextTick(
                                                        () ->
                                                            MotorTrackerPipe.getInstance()
                                                                .setCallbackForMotorPosition(
                                                                    new CallbackData<>(
                                                                        Turret.TURRET_MOTOR_NAME,
                                                                        (Integer ticks) ->
                                                                            ticks > (target - 5)
                                                                                && ticks
                                                                                    < (target + 5),
                                                                        () -> {
                                                                          robot.lift
                                                                              .setArmTwoPosition(
                                                                                  0.13);
                                                                          executorService.schedule(
                                                                              () ->
                                                                                  ExitPipe
                                                                                      .getInstance()
                                                                                      .onNextTick(
                                                                                          () ->
                                                                                              robot
                                                                                                  .lift
                                                                                                  .setArmOnePosition(
                                                                                                      firstJointOffset
                                                                                                          + -230)),
                                                                              100,
                                                                              TimeUnit
                                                                                  .MILLISECONDS);
                                                                        })));
                                              }));
                                }
                              }
                            }
                          }),
              100,
              TimeUnit.MILLISECONDS);
        },
        true,
        BooleanSurface.X);
    controller2.registerOnPressedCallback(
        () -> {
          robot.gripper.close();
          robot.intake.beginIntaking(); // Actually outtaking
          executorService.schedule(
              () -> ExitPipe.getInstance().onNextTick(robot.intake::stop),
              750,
              TimeUnit.MILLISECONDS);
          executorService.schedule(
              () ->
                  ExitPipe.getInstance()
                      .onNextTick(
                          () -> {
                            if (allianceHubMode.get()) {
                              double currentTurretPos = robot.turret.getState();
                              double target = -Turret.DEGREES_BACK * Turret.DEGREES_TO_TICKS;
                              if (Math.abs(currentTurretPos - target) < 5) {
                                robot.lift.setArmOnePosition(firstJointOffset + 550);
                                robot.lift.setArmTwoPosition(0.43);
                                robot.turret.turnCCWToBack();
                              } else {
                                robot.lift.setArmOnePosition(firstJointOffset + 550);
                                robot.lift.setArmTwoPosition(0.47);
                                MotorTrackerPipe.getInstance()
                                    .setCallbackForMotorPosition(
                                        new CallbackData<>(
                                            DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME,
                                            (Integer ticks) -> ticks > -5 && ticks < 5,
                                            () -> {
                                              robot.turret.turnCCWToBack();
                                              ExitPipe.getInstance()
                                                  .onNextTick(
                                                      () ->
                                                          MotorTrackerPipe.getInstance()
                                                              .setCallbackForMotorPosition(
                                                                  new CallbackData<>(
                                                                      Turret.TURRET_MOTOR_NAME,
                                                                      (Integer ticks) ->
                                                                          ticks > (target - 5)
                                                                              && ticks
                                                                                  < (target + 5),
                                                                      () ->
                                                                          robot.lift
                                                                              .setArmTwoPosition(
                                                                                  0.43))));
                                            }));
                              }
                            } else {
                              if (tippedMode.get()) {
                                // TODO: Tipped mode values
                              } else {
                                double currentTurretPos = robot.turret.getState();
                                double target = Turret.DEGREES_RIGHT * Turret.DEGREES_TO_TICKS;
                                if (Math.abs(currentTurretPos - target) < 5) {
                                  robot.lift.setArmOnePosition(firstJointOffset + 10);
                                  robot.lift.setArmTwoPosition(0.39);
                                  robot.turret.turnToRight();
                                } else {
                                  robot.lift.setArmOnePosition(firstJointOffset + 10);
                                  robot.lift.setArmTwoPosition(0.47);
                                  MotorTrackerPipe.getInstance()
                                      .setCallbackForMotorPosition(
                                          new CallbackData<>(
                                              DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME,
                                              (Integer ticks) ->
                                                  ticks > (10 - 5) && ticks < (10 + 5),
                                              () -> {
                                                robot.turret.turnToRight();
                                                ExitPipe.getInstance()
                                                    .onNextTick(
                                                        () ->
                                                            MotorTrackerPipe.getInstance()
                                                                .setCallbackForMotorPosition(
                                                                    new CallbackData<>(
                                                                        Turret.TURRET_MOTOR_NAME,
                                                                        (Integer ticks) ->
                                                                            ticks > (target - 5)
                                                                                && ticks
                                                                                    < (target + 5),
                                                                        () ->
                                                                            robot.lift
                                                                                .setArmTwoPosition(
                                                                                    0.39))));
                                              }));
                                }
                              }
                            }
                          }),
              100,
              TimeUnit.MILLISECONDS);
        },
        true,
        BooleanSurface.Y);
    controller2.registerOnPressedCallback(
        () -> {
          if (allianceHubMode.get()) {
            robot.gripper.close();
            executorService.schedule(
                () ->
                    ExitPipe.getInstance()
                        .onNextTick(
                            () -> {
                              double currentTurretPos = robot.turret.getState();
                              double target = -Turret.DEGREES_RIGHT * Turret.DEGREES_TO_TICKS;
                              if (Math.abs(currentTurretPos - target) < 5) {
                                robot.lift.setArmOnePosition(firstJointOffset + -540);
                                robot.lift.setArmTwoPosition(0);
                                robot.turret.turnToLeft();
                                robot.gripper.open();
                              } else {
                                robot.lift.setArmOnePosition(firstJointOffset);
                                robot.lift.setArmTwoPosition(0.47);
                                MotorTrackerPipe.getInstance()
                                    .setCallbackForMotorPosition(
                                        new CallbackData<>(
                                            DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME,
                                            (Integer ticks) -> ticks > -5 && ticks < 5,
                                            () -> {
                                              robot.turret.turnToLeft();
                                              ExitPipe.getInstance()
                                                  .onNextTick(
                                                      () ->
                                                          MotorTrackerPipe.getInstance()
                                                              .setCallbackForMotorPosition(
                                                                  new CallbackData<>(
                                                                      Turret.TURRET_MOTOR_NAME,
                                                                      (Integer ticks) ->
                                                                          ticks > (target - 5)
                                                                              && ticks
                                                                                  < (target + 5),
                                                                      () -> {
                                                                        robot.lift
                                                                            .setArmOnePosition(
                                                                                firstJointOffset
                                                                                    + -540);
                                                                        robot.lift
                                                                            .setArmTwoPosition(0);
                                                                        robot.gripper.open();
                                                                      })));
                                            }));
                              }
                            }),
                100,
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
                () ->
                    ExitPipe.getInstance()
                        .onNextTick(
                            () -> {
                              robot.lift.setArmOnePosition(firstJointOffset + 570);
                              robot.lift.setArmTwoPosition(0.3);
                              MotorTrackerPipe.getInstance()
                                  .setCallbackForMotorPosition(
                                      new CallbackData<>(
                                          DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME,
                                          (Integer ticks) -> ticks > (570 - 5) && ticks < (570 + 5),
                                          robot.turret::turnCCWToBack));
                            }),
                100,
                TimeUnit.MILLISECONDS);
          }
        },
        true,
        BooleanSurface.DPAD_UP);
    controller2.registerOnPressedCallback(
        () -> tippedMode.set(!tippedMode.get()), true, BooleanSurface.DPAD_DOWN);
    robot.turret.turnToFront();
    robot.lift.setArmOnePosition(firstJointOffset);
    hardwarePipeline.process(initializedHardware, new StateFilterResult(robotObject));
    hardwarePipeline.process(initializedHardware, new StateFilterResult(robotObject));
    hardwarePipeline.process(initializedHardware, new StateFilterResult(robotObject));
    hardwarePipeline.process(initializedHardware, new StateFilterResult(robotObject));
  }

  @Override
  public void onLoop() {
    double turnValue = controller1.rightStickX();
    double speed = halfSpeed.get() ? 0.5 : 1;
    robot.drivetrain.driveBySticks(
        controller1.leftStickX() * speed, controller1.leftStickY() * speed, turnValue * speed);
    if (controller2.leftTrigger() > 0.02 || controller2.rightTrigger() > 0.02) {

      robot.turret.turnToDegrees(
          (int)
              Math.round(
                  robot.turret.getState()
                      - (controller2.leftTrigger() * MAX_TURRET_ADJUSTMENT * 2)
                      + (controller2.rightTrigger() * MAX_TURRET_ADJUSTMENT)));
      previouslyTrimming = true;
    } else {
      if (previouslyTrimming) {
        robot.turret.turnToDegrees(
            MotorTrackerPipe.getInstance().getPositionOf(Turret.TURRET_MOTOR_NAME));
        previouslyTrimming = false;
      }
    }
    if (controller2.leftStickY() < -0.02 || controller2.leftStickY() > 0.02) {
      robot.lift.setArmOnePosition(
          firstJointOffset
              + (int)
                  Math.round(
                      robot.lift.getState().first
                          + (controller2.leftStickY() * MAX_FIRST_JOINT_ADJUSTMENT)));
    }
    if (controller2.rightStickY() < -0.02 || controller2.rightStickY() > 0.02) {
      robot.lift.setArmTwoPosition(
          robot.lift.getState().second
              + (-controller2.rightStickY() * MAX_SECOND_JOINT_ADJUSTMENT));
    }

    Log.d("WTF", "TURRET TARGET: " + robot.turret.getState());
    Log.d("WTF", "FIRST JOINT TARGET: " + robot.lift.getState().first);
  }

  @Override
  public void onStop() {}
}
