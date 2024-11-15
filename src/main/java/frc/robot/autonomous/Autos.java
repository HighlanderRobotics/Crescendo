package frc.robot.autonomous;

import com.choreo.lib.Choreo;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.carriage.CarriageSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.autoaim.AutoAim;

public class Autos {

    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private FeederSubsystem feeder;
    private SwerveSubsystem swerve;
    private CarriageSubsystem carriage;

    private boolean isInitialized = false;

    private static final Autos instance = new Autos();

    private Autos() {}

    public static Autos getInstance() {
        if (!instance.isInitialized) {
            // TODO: INITIALIZE
        }
        return instance;
    }

    // AUTOS -------------------------------------------------------------------

    public Command amp4Auto() {
        return Commands.sequence(
                autoFenderShot(),
                swerve
                        .runChoreoTraj(Choreo.getTrajectory("amp 4 local.1"), true)
                        .asProxy()
                        .deadlineWith(autoIntake()),
                autoIntake()
                        .until(() -> carriage.getBeambreak() || feeder.getFirstBeambreak())
                        .withTimeout(1.0),
                autoStaticAutoAim()
                        .andThen(Commands.print("Done with auto static auto aim"))
                        .beforeStarting(Commands.print("Before auto static auto aim!!!")),
                swerve
                        .runChoreoTraj(Choreo.getTrajectory("amp 4 local.2"))
                        .asProxy()
                        .deadlineWith(autoIntake()),
                autoIntake()
                        .until(() -> carriage.getBeambreak() || feeder.getFirstBeambreak())
                        .withTimeout(1.0),
                autoStaticAutoAim(),
                swerve
                        .runChoreoTraj(Choreo.getTrajectory("amp 4 local.3"))
                        .asProxy()
                        .deadlineWith(autoIntake()),
                autoIntake().until(() -> carriage.getBeambreak() || feeder.getFirstBeambreak()),
                autoStaticAutoAim()
        );
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
                        // FIXME
                        staticAutoAim(6.0)
                                .deadlineWith(
                                        intake.runVoltageCmd(0, 0).asProxy(), carriage.runVoltageCmd(0).asProxy())
                                .withTimeout(2.0)
                                .unless(() -> !feeder.getFirstBeambreak()))
                .asProxy()
                .withTimeout(4.0);
    }



}
