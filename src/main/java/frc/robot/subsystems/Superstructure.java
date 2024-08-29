// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.Robot.Target;
import frc.robot.subsystems.carriage.CarriageSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.autoaim.ShotData;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public static enum SuperState {
    IDLE,
    INTAKE,
    INDEX_SHOOTER,
    READY_INDEXED_SHOOTER,
    INDEX_CARRIAGE,
    READY_INDEXED_CARRIAGE,
    REVERSE_INDEX_CARRIAGE,
    PRESHOOT,
    PREFEED,
    PRESUB,
    SHOOT,
    PREAMP,
    AMP,
    SPIT,
    PRECLIMB,
    CLIMB,
    HOME_ELEVATOR,
    HOME_SHOOTER
  }

  private final PivotSubsystem pivot;
  private final FlywheelSubsystem leftFlywheel;
  private final FlywheelSubsystem rightFlywheel;
  private final FeederSubsystem feeder;
  private final CarriageSubsystem carriage;
  private final IntakeSubsystem intake;
  private final ElevatorSubsystem elevator;

  private final Supplier<Target> target;
  private final Supplier<Pose2d> pose;
  private final Supplier<ShotData> shot;
  private final Supplier<ChassisSpeeds> chassisVel;

  /** Also triggered by scoreReq */
  private final Trigger prescoreReq;

  private final Trigger scoreReq;
  private final Trigger intakeReq;
  private final Trigger preClimbReq;
  private final Trigger climbConfReq;
  private final Trigger climbCanReq;

  // This feels messy
  private boolean scoreOverride = false;
  private boolean intakeOverride = false;

  private SuperState state = SuperState.IDLE;
  private SuperState prevState = SuperState.IDLE;
  private Map<SuperState, Trigger> stateTriggers = new HashMap<SuperState, Trigger>();

  private Timer stateTimer = new Timer();

  public Superstructure(
      PivotSubsystem pivot,
      FlywheelSubsystem leftFlywheel,
      FlywheelSubsystem rightFlywheel,
      FeederSubsystem feeder,
      CarriageSubsystem carriage,
      IntakeSubsystem intake,
      ElevatorSubsystem elevator,
      Supplier<Target> target,
      Supplier<Pose2d> pose,
      Supplier<ShotData> shot,
      Supplier<ChassisSpeeds> chassisVel,
      Trigger scoreReq,
      Trigger prescoreReq,
      Trigger intakeReq,
      Trigger climbReq,
      Trigger climbConfReq,
      Trigger climbCanReq) {
    this.pivot = pivot;
    this.leftFlywheel = leftFlywheel;
    this.rightFlywheel = rightFlywheel;
    this.feeder = feeder;
    this.carriage = carriage;
    this.intake = intake;
    this.elevator = elevator;

    this.target = target;
    this.pose = pose;
    this.shot = shot;
    this.chassisVel = chassisVel;

    this.prescoreReq = prescoreReq.or(scoreReq);
    this.scoreReq = scoreReq.or(() -> scoreOverride);
    this.intakeReq = intakeReq.or(() -> intakeOverride);
    this.preClimbReq = climbReq;
    this.climbConfReq = climbConfReq;
    this.climbCanReq = climbCanReq;

    stateTimer.start();

    for (var state : SuperState.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state));
    }

    configureStateTransitionCommands();
  }

  /** This file is not a subsystem, so this MUST be called manually. */
  public void periodic() {
    Logger.recordOutput("Superstructure State", state);
  }

  private void configureStateTransitionCommands() {
    // IDLE stops everything or sets it to 0
    // Should this just force everything to go back to default commands?
    stateTriggers
        .get(SuperState.IDLE)
        .onTrue(pivot.setMinCmd())
        .onTrue(leftFlywheel.setVoltageCmd(0.0))
        .onTrue(rightFlywheel.setVoltageCmd(0.0))
        .onTrue(elevator.setExtensionCmd(0.0))
        .onTrue(intake.stop())
        .onTrue(carriage.setVoltageCmd(0.0))
        .onTrue(feeder.setVelocityCmd(0.0))
        // State Transitions
        // Rerun the intake and indexing sequence if the note moves during IDLE
        .and(carriage.beambreakTrig.or(feeder.beambreakTrig).or(intakeReq))
        .onTrue(this.setState(SuperState.INTAKE));
    stateTriggers.get(SuperState.IDLE)
    .or(stateTriggers.get(SuperState.READY_INDEXED_SHOOTER))
    .or(stateTriggers.get(SuperState.READY_INDEXED_CARRIAGE))
    .and(preClimbReq).onTrue(this.setState(SuperState.SPIT));

    stateTriggers
        .get(SuperState.INTAKE)
        .whileTrue(intake.intake())
        .onFalse(intake.stop())
        .whileTrue(carriage.setVoltageCmd(CarriageSubsystem.INDEXING_VOLTAGE))
        // State Transition
        .and(carriage.beambreakTrig.or(feeder.beambreakTrig))
        .onTrue(
            this.setState(
                target.get().isSpeakerAlike()
                    ? SuperState.INDEX_SHOOTER
                    : SuperState.INDEX_CARRIAGE));
    stateTriggers
        .get(SuperState.INTAKE)
        .and(intakeReq.negate())
        .onTrue(this.setState(SuperState.IDLE));

    stateTriggers
        .get(SuperState.INDEX_CARRIAGE)
        .onTrue(elevator.setExtensionCmd(0.0))
        .whileTrue(carriage.indexForwardsCmd())
        // State Transition
        .and(carriage.isIndexedTrig)
        .onTrue(this.setState(SuperState.READY_INDEXED_CARRIAGE));

    // This state is meant to just hold the ring until we score or re-index
    stateTriggers
        .get(SuperState.READY_INDEXED_CARRIAGE)
        .whileTrue(carriage.stop())
        // Reject any additional rings
        .onTrue(intake.setVelocityCmd(-50.0, -30.0).withTimeout(1.0))
        .and(prescoreReq)
        .and(() -> target.get() == Target.AMP)
        .onTrue(this.setState(SuperState.PREAMP));
    stateTriggers
        .get(SuperState.READY_INDEXED_CARRIAGE)
        .and(() -> target.get() != Target.AMP)
        .onTrue(this.setState(SuperState.INDEX_SHOOTER));

    stateTriggers
        .get(SuperState.PREAMP)
        .whileTrue(elevator.setExtensionCmd(ElevatorSubsystem.AMP_EXTENSION_METERS))
        // State Transition
        .and(() -> elevator.getExtensionMeters() > 0.95 * ElevatorSubsystem.AMP_EXTENSION_METERS)
        .and(scoreReq)
        .onTrue(this.setState(SuperState.AMP));

    stateTriggers
        .get(SuperState.AMP)
        .whileTrue(elevator.setExtensionCmd(ElevatorSubsystem.AMP_EXTENSION_METERS))
        .whileTrue(carriage.setVoltageCmd(-3.0))
        // State Transition
        .and(carriage.beambreakTrig.negate().debounce(0.5)) // Beambreak not triggered for 0.5s
        .onTrue(this.setState(SuperState.IDLE));

    stateTriggers
        .get(SuperState.INDEX_SHOOTER)
        .whileTrue(elevator.setExtensionCmd(0.0))
        .whileTrue(pivot.setMinCmd())
        .whileTrue(carriage.setIndexingVoltageCmd())
        .whileTrue(feeder.indexCmd())
        .and(feeder.beambreakTrig)
        .onTrue(this.setState(SuperState.READY_INDEXED_SHOOTER));

    stateTriggers
        .get(SuperState.READY_INDEXED_SHOOTER)
        // Reject any additional rings
        .onTrue(intake.setVelocityCmd(-50.0, -30.0).withTimeout(1.0));
    // State Transitions
    stateTriggers
        .get(SuperState.READY_INDEXED_SHOOTER)
        .and(prescoreReq)
        .and(() -> target.get() == Target.SPEAKER)
        .onTrue(this.setState(SuperState.PRESHOOT));
    stateTriggers
        .get(SuperState.READY_INDEXED_SHOOTER)
        .and(prescoreReq)
        .and(() -> target.get() == Target.FEED)
        .onTrue(this.setState(SuperState.PREFEED));
    stateTriggers
        .get(SuperState.READY_INDEXED_SHOOTER)
        .and(prescoreReq)
        .and(() -> target.get() == Target.SUBWOOFER)
        .onTrue(this.setState(SuperState.PRESUB));
    stateTriggers
        .get(SuperState.READY_INDEXED_SHOOTER)
        .and(() -> target.get() == Target.AMP)
        .onTrue(this.setState(SuperState.REVERSE_INDEX_CARRIAGE));

    stateTriggers
        .get(SuperState.REVERSE_INDEX_CARRIAGE)
        .whileTrue(elevator.setExtensionCmd(0.0))
        .whileTrue(pivot.setMinCmd())
        .whileTrue(feeder.setVelocityCmd(-FeederSubsystem.INDEXING_VELOCITY))
        .whileTrue(carriage.setVoltageCmd(-CarriageSubsystem.INDEXING_VOLTAGE))
        .whileTrue(
            Commands.waitUntil(carriage.beambreakTrig)
                .andThen(
                    intake.setVelocityCmd(-50.0, -50.0).until(carriage.beambreakTrig.negate().debounce(0.25)),
                    intake.setVelocityCmd(50.0, 30.0)))
        .and(carriage.beambreakTrig.negate().debounce(0.4))
        .onTrue(this.setState(SuperState.INTAKE));

    // TODO add cancellation behavior
    stateTriggers
        .get(SuperState.PRESHOOT)
        .whileTrue(pivot.setPivotSetpoint(shot.get().getRotation()))
        .whileTrue(leftFlywheel.setVelocity(shot.get().getLeftRPS()))
        .whileTrue(rightFlywheel.setVelocity(shot.get().getRightRPS()))
        .whileTrue(feeder.stop())
        .and(scoreReq)
        .and(pivot::isAtGoal)
        .and(leftFlywheel::isAtGoal)
        .and(rightFlywheel::isAtGoal)
        .and(
            () ->
                MathUtil.isNear(
                    pose.get()
                        .getTranslation()
                        .minus(FieldConstants.getSpeaker().getTranslation())
                        .getAngle()
                        .getDegrees(),
                    pose.get().getRotation().getDegrees(),
                    pose.get().minus(FieldConstants.getSpeaker()).getTranslation().getNorm() < 3.0
                        ? 5.0
                        : 3.0))
        .and(() -> chassisVel.get().omegaRadiansPerSecond < Units.degreesToRadians(90.0))
        .onTrue(this.setState(SuperState.SHOOT));

    stateTriggers
        .get(SuperState.PREFEED)
        .whileTrue(pivot.setPivotSetpoint(AutoAim.FEED_SHOT.getRotation()))
        .whileTrue(leftFlywheel.setVelocity(AutoAim.FEED_SHOT.getLeftRPS()))
        .whileTrue(rightFlywheel.setVelocity(AutoAim.FEED_SHOT.getRightRPS()))
        .whileTrue(feeder.stop())
        .and(scoreReq)
        .and(pivot::isAtGoal)
        .and(leftFlywheel::isAtGoal)
        .and(rightFlywheel::isAtGoal)
        // Make sure we are in midfield
        .and(() -> pose.get().getX() > 6.3 && pose.get().getX() < 10.2)
        // Make sure we aren't moving fast
        .and(
            () ->
                0.5
                    > Math.hypot(
                        chassisVel.get().vxMetersPerSecond, chassisVel.get().vyMetersPerSecond))
        .onTrue(this.setState(SuperState.SHOOT));

    stateTriggers
        .get(SuperState.PRESUB)
        .whileTrue(pivot.setPivotSetpoint(AutoAim.FENDER_SHOT.getRotation()))
        .whileTrue(leftFlywheel.setVelocity(AutoAim.FENDER_SHOT.getLeftRPS()))
        .whileTrue(rightFlywheel.setVelocity(AutoAim.FENDER_SHOT.getRightRPS()))
        .whileTrue(feeder.stop())
        .and(scoreReq)
        .and(pivot::isAtGoal)
        .and(leftFlywheel::isAtGoal)
        .and(rightFlywheel::isAtGoal)
        .onTrue(this.setState(SuperState.SHOOT));

    stateTriggers
        .get(SuperState.SHOOT)
        // Explicitly maintain previous state
        // This should happen by default anyways
        .whileTrue(pivot.run(() -> {}))
        .whileTrue(leftFlywheel.run(() -> {}))
        .whileTrue(rightFlywheel.run(() -> {}))
        .whileTrue(feeder.setVelocityCmd(FeederSubsystem.INDEXING_VELOCITY))
        .and(() -> stateTimer.get() > 0.5)
        .onTrue(this.setState(SuperState.IDLE));

    stateTriggers
        .get(SuperState.SPIT)
        .whileTrue(pivot.setPivotSetpoint(PivotSubsystem.MIN_ANGLE))
        .whileTrue(leftFlywheel.setVelocity(20.0))
        .whileTrue(rightFlywheel.setVelocity(20.0))
        .whileTrue(feeder.setVelocityCmd(FeederSubsystem.INDEXING_VELOCITY))
        .whileTrue(carriage.setVoltageCmd(CarriageSubsystem.INDEXING_VOLTAGE))
        .and(() -> feeder.getFirstBeambreak() || carriage.getBeambreak())
        .debounce(0.4)
        .onTrue(this.setState(SuperState.PRECLIMB));

    stateTriggers
        .get(SuperState.PRECLIMB)
        .whileTrue(elevator.setExtensionCmd(ElevatorSubsystem.CLIMB_EXTENSION_METERS))
        .and(elevator::isAtClimb)
        .and(climbConfReq)
        .debounce(0.25)
        .onTrue(this.setState(SuperState.CLIMB));
    stateTriggers
        .get(SuperState.PRECLIMB)
        .and(climbCanReq)
        // Debounce in case we cancel from pull into preclimb
        .debounce(0.4)
        .onTrue(this.setState(SuperState.IDLE));

    stateTriggers
        .get(SuperState.CLIMB)
        .whileTrue(elevator.climbRetractAndLock())
        // This is dangerous bc we cant confirm if the climb unlatches
        // Consider better tracking for if we are likely to be latched
        .and(climbCanReq)
        // Switching to homing is the best way to try to put it low to unlatch
        .onTrue(
            Commands.either(
                this.setState(SuperState.HOME_ELEVATOR),
                this.setState(SuperState.PRECLIMB),
                // If we are low, try to unlatch. If we are high, try to extend.
                () -> elevator.getExtensionMeters() < 0.25));

    stateTriggers
        .get(SuperState.HOME_ELEVATOR)
        .whileTrue(elevator.runCurrentZeroing())
        .and(() -> !elevator.getCurrentCommand().getName().equals("Homing"))
        .debounce(0.1)
        .onTrue(this.setState(SuperState.IDLE));

    // Since current zeroing doesnt work this is kinda iffy
    // Should consider not using a state for this
    stateTriggers
        .get(SuperState.HOME_SHOOTER)
        .whileTrue(pivot.resetPosition(PivotSubsystem.MIN_ANGLE))
        .debounce(0.1)
        .onTrue(this.setState(SuperState.IDLE));
  }

  /** Force sets the current state */
  private Command setState(SuperState state) {
    return Commands.runOnce(
        () -> {
          this.prevState = state;
          this.state = state;
          stateTimer.restart();
        });
  }

  /** Request the robot to score in the current target */
  public Command score() {
    return Commands.runOnce(() -> scoreOverride = true)
        .andThen(
            Commands.waitUntil(
                () ->
                    this.state == SuperState.IDLE
                        && (this.prevState == SuperState.SHOOT
                            || this.prevState == SuperState.AMP)))
        .finallyDo(() -> scoreOverride = false);
  }

  /** Request the robot to intake */
  public Command intake() {
    return Commands.runOnce(() -> intakeOverride = true)
        .andThen(Commands.waitUntil(() -> this.prevState == SuperState.INTAKE))
        .finallyDo(() -> intakeOverride = false);
  }

  public Command homeElevator() {
    return Commands.waitUntil(() -> state == SuperState.IDLE)
        .andThen(this.setState(SuperState.HOME_ELEVATOR));
  }
}
