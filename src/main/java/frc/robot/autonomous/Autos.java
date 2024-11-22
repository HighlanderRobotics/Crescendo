package frc.robot.autonomous;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.carriage.CarriageSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.autoaim.AutoAim;
import java.util.function.Supplier;

public class Autos {

  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private FeederSubsystem feeder;
  private SwerveSubsystem swerve;
  private CarriageSubsystem carriage;

  private Supplier<Command> staticAutoAim;

  private static Autos instance = null;

  private Autos(
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      FeederSubsystem feeder,
      SwerveSubsystem swerve,
      CarriageSubsystem carriage,
      Supplier<Command> staticAutoAim) {
    this.intake = intake;
    this.shooter = shooter;
    this.feeder = feeder;
    this.swerve = swerve;
    this.carriage = carriage;
    this.staticAutoAim = staticAutoAim;
  }

  public static void init(
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      SwerveSubsystem swerveSubsystem,
      CarriageSubsystem carriageSubsystem,
      Supplier<Command> staticAutoAim) {
    instance =
        new Autos(
            intakeSubsystem,
            shooterSubsystem,
            feederSubsystem,
            swerveSubsystem,
            carriageSubsystem,
            staticAutoAim);
  }

  public static Autos getInstance() {
    if (instance == null) {
      throw new NullPointerException("Instance is not initialized!");
    }
    return instance;
  }

  // AUTOS -------------------------------------------------------------------

  public Command amp4() {
    return Commands.sequence(
        // Shoot preload
        autoFenderShot(),
        runChoreoTrajWithIntake("amp 4 local.1", true),
        autoIntake()
            .until(() -> carriage.getBeambreak() || feeder.getFirstBeambreak())
            .withTimeout(1.0),
        autoStaticAutoAim()
            .andThen(Commands.print("Done with auto static auto aim"))
            .beforeStarting(Commands.print("Before auto static auto aim!!!")),
        runChoreoTrajWithIntake("amp 4 local.2", false),
        autoIntake()
            .until(() -> carriage.getBeambreak() || feeder.getFirstBeambreak())
            .withTimeout(1.0),
        autoStaticAutoAim(),
        runChoreoTrajWithIntake("amp 4 local.3", false),
        autoIntake().until(() -> carriage.getBeambreak() || feeder.getFirstBeambreak()),
        autoStaticAutoAim());
  }

  public Command source3() {
    return Commands.sequence(
        autoFenderShot(),
        runChoreoTrajWithIntake("source 3.1", true),
        autoIntake()
            .raceWith(
                Commands.sequence(
                    Commands.waitSeconds(0.25),
                    Commands.waitUntil(
                        () -> carriage.getBeambreak() || feeder.getFirstBeambreak())))
            .withTimeout(1.0),
        autoStaticAutoAim().unless(() -> !feeder.getFirstBeambreak()),
        runChoreoTrajWithIntake("source 3.2", false),
        autoIntake()
            .raceWith(
                Commands.sequence(
                    Commands.waitSeconds(0.25),
                    Commands.waitUntil(
                        () -> carriage.getBeambreak() || feeder.getFirstBeambreak())))
            .withTimeout(1.0),
        autoStaticAutoAim());
  }

  public Command source4() {
    return Commands.sequence(
        autoFenderShot(),
        runChoreoTrajWithIntake("source 4.1", true),
        autoIntake()
            .raceWith(
                Commands.sequence(
                    Commands.waitSeconds(0.25),
                    Commands.waitUntil(
                        () -> carriage.getBeambreak() || feeder.getFirstBeambreak())))
            .withTimeout(1.0),
        autoStaticAutoAim().unless(() -> !feeder.getFirstBeambreak()),
        runChoreoTrajWithIntake("source 4.2", false),
        autoIntake()
            .raceWith(
                Commands.sequence(
                    Commands.waitSeconds(0.25),
                    Commands.waitUntil(
                        () -> carriage.getBeambreak() || feeder.getFirstBeambreak())))
            .withTimeout(1.0),
        autoStaticAutoAim().unless(() -> !feeder.getFirstBeambreak()),
        runChoreoTrajWithIntake("source 4.3", false),
        autoIntake()
            .raceWith(
                Commands.sequence(
                    Commands.waitSeconds(0.25),
                    Commands.waitUntil(
                        () -> carriage.getBeambreak() || feeder.getFirstBeambreak())))
            .withTimeout(1.0),
        autoStaticAutoAim());
  }

  public Command amp5() {
    return Commands.sequence(
        autoFenderShot(),
        runChoreoTrajWithIntake("amp 5.1", true),
        autoStaticAutoAim(),
        swerve
            .runChoreoTraj(Choreo.getTrajectory("amp 5.2"))
            .asProxy()
            .deadlineWith(Commands.waitSeconds(1.0).andThen(autoIntake())),
        autoStaticAutoAim(),
        runChoreoTrajWithIntake("amp 5.3", false),
        autoIntake()
            .until(() -> carriage.getBeambreak() || feeder.getFirstBeambreak())
            .withTimeout(1.0),
        autoStaticAutoAim(),
        runChoreoTrajWithIntake("amp 5.4", false),
        autoIntake()
            .until(() -> carriage.getBeambreak() || feeder.getFirstBeambreak())
            .withTimeout(1.0),
        autoStaticAutoAim());
  }

  public Command center4() {
    return Commands.sequence(
        autoFenderShot(),
        runChoreoTrajWithIntake("center 4.1", true),
        autoIntake()
            .raceWith(
                Commands.sequence(
                    Commands.waitSeconds(0.25),
                    Commands.waitUntil(
                        () -> carriage.getBeambreak() || feeder.getFirstBeambreak())))
            .withTimeout(1.0),
        autoStaticAutoAim().unless(() -> !feeder.getFirstBeambreak()),
        runChoreoTrajWithIntake("center 4.2", false),
        autoIntake()
            .raceWith(
                Commands.sequence(
                    Commands.waitSeconds(0.25),
                    Commands.waitUntil(
                        () -> carriage.getBeambreak() || feeder.getFirstBeambreak())))
            .withTimeout(1.0),
        autoStaticAutoAim().unless(() -> !feeder.getFirstBeambreak()),
        runChoreoTrajWithIntake("center 4.3", false),
        autoIntake()
            .raceWith(
                Commands.sequence(
                    Commands.waitSeconds(0.25),
                    Commands.waitUntil(
                        () -> carriage.getBeambreak() || feeder.getFirstBeambreak())))
            .withTimeout(1.0),
        autoStaticAutoAim());
  }

  public Command lineTest() {
    ChoreoTrajectory traj = Choreo.getTrajectory("line test");
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              if (DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
                swerve.setPose(traj.flipped().getInitialPose());
              } else {
                swerve.setPose(traj.getInitialPose());
              }
            }),
        swerve.runChoreoTraj(traj).repeatedly());
  }

  // UTIL COMMANDS ---------------------------------------------------------------

  private Command autoFenderShot() {
    return shooter
        .runStateCmd(
            AutoAim.FENDER_SHOT.getRotation(),
            AutoAim.FENDER_SHOT.getLeftRPS(),
            AutoAim.FENDER_SHOT.getRightRPS())
        .raceWith(
            feeder
                .runVelocityCmd(0.0)
                .until(shooter::isAtGoal)
                .andThen(
                    feeder
                        .runVelocityCmd(FeederSubsystem.INDEXING_VELOCITY)
                        .raceWith(
                            Commands.waitUntil(() -> !feeder.getFirstBeambreak())
                                .andThen(Commands.waitSeconds(0.1)))));
  }

  private Command autoIntake() {
    return Commands.parallel(
        intake
            .runVelocityCmd(50.0, 30.0)
            .until(() -> carriage.getBeambreak() || feeder.getFirstBeambreak())
            .asProxy(),
        feeder.indexCmd().asProxy(),
        carriage
            .runVoltageCmd(CarriageSubsystem.INDEXING_VOLTAGE)
            .until(feeder::getFirstBeambreak)
            .asProxy(),
        shooter
            .runStateCmd(
                () ->
                    Rotation2d.fromDegrees(
                        MathUtil.clamp(
                            AutoAim.shotMap
                                .get(swerve.getDistanceToSpeaker())
                                .getRotation()
                                .getDegrees(),
                            0,
                            20)),
                () -> 0.0,
                () -> 0.0)
            .until(feeder::getFirstBeambreak)
            .unless(feeder::getFirstBeambreak)
            .andThen(
                shooter.runStateCmd(
                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRotation(),
                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getLeftRPS(),
                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRightRPS()))
            .asProxy());
  }

  private Command autoStaticAutoAim() {
    return feeder
        .indexCmd()
        .deadlineWith(
            swerve.runVelocityCmd(() -> new ChassisSpeeds()),
            Commands.repeatingSequence(
                shooter
                    .runFlywheelsCmd(() -> 0.0, () -> 0.0)
                    .unless(() -> feeder.getFirstBeambreak() && swerve.getDistanceToSpeaker() < 8.0)
                    .until(() -> feeder.getFirstBeambreak() && swerve.getDistanceToSpeaker() < 8.0),
                shooter.runStateCmd(
                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRotation(),
                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getLeftRPS(),
                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRightRPS())))
        .until(() -> feeder.getFirstBeambreak())
        .unless(() -> feeder.getFirstBeambreak())
        .withTimeout(1.0)
        .andThen(
            staticAutoAim
                .get()
                .deadlineWith(
                    intake.runVoltageCmd(0, 0).asProxy(), carriage.runVoltageCmd(0).asProxy())
                .withTimeout(2.0)
                .unless(() -> !feeder.getFirstBeambreak()))
        .asProxy()
        .withTimeout(4.0);
  }

  private Command runChoreoTrajWithIntake(String trajName, boolean resetPose) {
    return swerve
        .runChoreoTraj(Choreo.getTrajectory(trajName), resetPose)
        .asProxy()
        .deadlineWith(autoIntake());
  }
}
