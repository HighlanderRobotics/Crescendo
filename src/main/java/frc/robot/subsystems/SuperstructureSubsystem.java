// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.Target;
import frc.robot.subsystems.carriage.CarriageSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.autoaim.ShotData;

public class SuperstructureSubsystem extends SubsystemBase {
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

  /** Also triggered by scoreReq */
  private final Trigger prescoreReq;
  private final Trigger scoreReq;
  private final Trigger intakeReq;
  private final Trigger climbReq;

  private SuperState state = SuperState.IDLE;
  private Map<SuperState, Trigger> stateTriggers;

  private Timer stateTimer = new Timer();

  public SuperstructureSubsystem(
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
    Trigger scoreReq,
    Trigger prescoreReq,
    Trigger intakeReq,
    Trigger climbReq
  ) {
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
    
    this.prescoreReq = prescoreReq.or(scoreReq);
    this.scoreReq = scoreReq;
    this.intakeReq = intakeReq;
    this.climbReq = climbReq;

    stateTimer.start();

    for (var state : SuperState.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state));
    }

    configureStateTransitionCommands();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Superstructure State", state);
  }

  private void configureStateTransitionCommands() {
    // IDLE stops everything or sets it to 0
    // Should this just force everything to go back to default commands?
    stateTriggers.get(SuperState.IDLE)
      .onTrue(pivot.setMinCmd())
      .onTrue(leftFlywheel.setVoltageCmd(0.0))
      .onTrue(rightFlywheel.setVoltageCmd(0.0))
      .onTrue(elevator.setExtensionCmd(0.0))
      .onTrue(intake.stop())
      .onTrue(carriage.setVoltageCmd(0.0))
      .onTrue(feeder.setVelocityCmd(0.0))
      // State Transitions
      .and(
        carriage.beambreakTrig.or(feeder.beambreakTrig)
        .or(intakeReq)
        )
      .onTrue(this.setState(SuperState.INTAKE));
    stateTriggers.get(SuperState.IDLE).and(climbReq).onTrue(this.setState(SuperState.SPIT));
    
    stateTriggers.get(SuperState.INTAKE)
      .onTrue(intake.intake())
      .onFalse(intake.stop())
      .whileTrue(carriage.setVoltageCmd(CarriageSubsystem.INDEXING_VOLTAGE))
      // State Transition
      .onFalse(this.setState(target.get().isSpeakerAlike() ? SuperState.INDEX_SHOOTER : SuperState.INDEX_CARRIAGE));
    
    stateTriggers.get(SuperState.INDEX_CARRIAGE)
    .whileTrue(elevator.setExtensionCmd(0.0))
      .whileTrue(carriage.indexForwardsCmd())
      // State Transition
      .and(carriage.isIndexedTrig)
      .onTrue(this.setState(SuperState.READY_INDEXED_CARRIAGE));
    
    // This state is meant to just hold the ring until we score or re-index
    stateTriggers.get(SuperState.READY_INDEXED_CARRIAGE)
      .whileTrue(carriage.stop())
      .and(prescoreReq)
      .and(() -> target.get() == Target.AMP)
      .onTrue(this.setState(SuperState.PREAMP));
      
    stateTriggers.get(SuperState.PREAMP)
      .whileTrue(elevator.setExtensionCmd(ElevatorSubsystem.AMP_EXTENSION_METERS))
      // State Transition
      .and(() -> elevator.getExtensionMeters() > 0.95 * ElevatorSubsystem.AMP_EXTENSION_METERS)
      .and(scoreReq)
      .onTrue(this.setState(SuperState.AMP));
      
    stateTriggers.get(SuperState.AMP)
      .whileTrue(elevator.setExtensionCmd(ElevatorSubsystem.AMP_EXTENSION_METERS))
      .whileTrue(carriage.setVoltageCmd(-3.0))
      // State Transition
      .and(carriage.beambreakTrig.negate().debounce(0.5)) // Beambreak not triggered for 0.5s
      .onTrue(this.setState(SuperState.IDLE));
  
    stateTriggers.get(SuperState.INDEX_SHOOTER)
      .whileTrue(elevator.setExtensionCmd(0.0))
      .whileTrue(pivot.setMinCmd())
      .whileTrue(carriage.setIndexingVoltageCmd())
      .whileTrue(feeder.indexCmd())
      .and(feeder.beambreakTrig)
      .onTrue(this.setState(SuperState.READY_INDEXED_SHOOTER));
    // State Transitions
    stateTriggers.get(SuperState.READY_INDEXED_SHOOTER).and(prescoreReq).and(() -> target.get() == Target.SPEAKER).onTrue(this.setState(SuperState.PRESHOOT));
    stateTriggers.get(SuperState.READY_INDEXED_SHOOTER).and(prescoreReq).and(() -> target.get() == Target.FEED).onTrue(this.setState(SuperState.PREFEED));
    stateTriggers.get(SuperState.READY_INDEXED_SHOOTER).and(prescoreReq).and(() -> target.get() == Target.SUBWOOFER).onTrue(this.setState(SuperState.PRESUB));
    stateTriggers.get(SuperState.READY_INDEXED_SHOOTER).and(() -> target.get() == Target.AMP).onTrue(this.setState(SuperState.REVERSE_INDEX_CARRIAGE));

    stateTriggers.get(SuperState.REVERSE_INDEX_CARRIAGE)
      .whileTrue(elevator.setExtensionCmd(0.0))
      .whileTrue(pivot.setMinCmd())
      .whileTrue(feeder.setVelocityCmd(-FeederSubsystem.INDEXING_VELOCITY))
      .whileTrue(carriage.indexBackwardsCmd())
      .and(carriage.isIndexedTrig)
      .onTrue(this.setState(SuperState.READY_INDEXED_CARRIAGE));

    stateTriggers.get(SuperState.PRESHOOT)
      .whileTrue(pivot.setPivotSetpoint(shot.get().getRotation()))
      .whileTrue(leftFlywheel.setVelocity(shot.get().getLeftRPS()))
      .whileTrue(rightFlywheel.setVelocity(shot.get().getRightRPS()))
      .whileTrue(feeder.stop())
      .and(scoreReq)
      .and(pivot::isAtGoal)
      .and(leftFlywheel::isAtGoal)
      .and(rightFlywheel::isAtGoal)
      .onTrue(this.setState(SuperState.SHOOT));

    stateTriggers.get(SuperState.PREFEED)
      .whileTrue(pivot.setPivotSetpoint(AutoAim.FEED_SHOT.getRotation()))
      .whileTrue(leftFlywheel.setVelocity(AutoAim.FEED_SHOT.getLeftRPS()))
      .whileTrue(rightFlywheel.setVelocity(AutoAim.FEED_SHOT.getRightRPS()))
      .whileTrue(feeder.stop())
      .and(scoreReq)
      .and(pivot::isAtGoal)
      .and(leftFlywheel::isAtGoal)
      .and(rightFlywheel::isAtGoal)
      .onTrue(this.setState(SuperState.SHOOT));

    stateTriggers.get(SuperState.PREFEED)
      .whileTrue(pivot.setPivotSetpoint(AutoAim.FENDER_SHOT.getRotation()))
      .whileTrue(leftFlywheel.setVelocity(AutoAim.FENDER_SHOT.getLeftRPS()))
      .whileTrue(rightFlywheel.setVelocity(AutoAim.FENDER_SHOT.getRightRPS()))
      .whileTrue(feeder.stop())
      .and(scoreReq)
      .and(pivot::isAtGoal)
      .and(leftFlywheel::isAtGoal)
      .and(rightFlywheel::isAtGoal)
      .onTrue(this.setState(SuperState.SHOOT));
    
    stateTriggers.get(SuperState.SHOOT)
      // maintain previous state
      .whileTrue(pivot.run(() -> {}))
      .whileTrue(leftFlywheel.run(() -> {}))
      .whileTrue(rightFlywheel.run(() -> {}))
      .whileTrue(feeder.setVelocityCmd(FeederSubsystem.INDEXING_VELOCITY))
      .and(() -> stateTimer.get() > 0.5)
      .onTrue(this.setState(SuperState.IDLE));
  }

  /** Force sets the current state */
  private Command setState(SuperState state) {
    return Commands.runOnce(() -> {this.state = state; stateTimer.restart();});
  }
}
