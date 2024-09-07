package frc.robot.utils.mapleUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.PHoenixOdometryThread;
import frc.robot.utils.mapleUtils.CustomMaths.GeometryConvertor;
import frc.robot.utils.mapleUtils.MapleTimeUtils;
import org.dyn4j.geometry.Vector2;

import java.util.Arrays;
import java.util.function.Consumer;

import static frc.robot.DriveTrainConstants.*;

/**
 * simulates the behavior of our robot
 * it has all the physics behavior as a simulated holonomic chassis
 * in addition to that, it simulates the swerve module behaviors
 * the class is like the bridge between ModuleIOSim and HolonomicChassisSimulation
 * it reads the motor power from ModuleIOSim
 * and feed the result of the physics simulation back to ModuleIOSim, to simulate the odometry encoders' readings
 * */
public class SwerveDriveSimulation extends HolonomicChassisSimulation {
    private final GyroIOSim gyroIOSim;
    private final ModuleIOSim[] modules;
    private final Consumer<Pose2d> resetOdometryCallBack;
    public SwerveDriveSimulation(
            GyroIOSim gyroIOSim,
            ModuleIOSim frontLeft, ModuleIOSim frontRight, ModuleIOSim backLeft, ModuleIOSim backRight,
            Pose2d startingPose,
            Consumer<Pose2d> resetOdometryCallBack
    ) {
        super(new RobotSimulationProfile(

        ), startingPose);
        this.gyroIOSim = gyroIOSim;
        this.modules = new ModuleIOSim[] {frontLeft, frontRight, backLeft, backRight};
        this.resetOdometryCallBack = resetOdometryCallBack;
        resetOdometryToActualRobotPose();
        System.out.println("swerve drive sim profile: " + new RobotSimulationProfile());
    }

    public void resetOdometryToActualRobotPose() {
        resetOdometryCallBack.accept(getObjectOnFieldPose2d());
    }

    @Override
    public void updateSimulationSubTick(int tickNum, double tickSeconds) {
        for (int i = 0; i < modules.length; i++)
            moduleSimulationSubTick(
                    getObjectOnFieldPose2d(),
                    modules[i],
                    MODULE_TRANSLATIONS[i],
                    tickNum, tickSeconds
            );

        simulateFrictionForce();

        gyroSimulationSubTick(
                super.getObjectOnFieldPose2d().getRotation(),
                super.getAngularVelocity(),
                tickNum
        );
    }

    private void moduleSimulationSubTick(
            Pose2d robotWorldPose,
            ModuleIOSim module,
            Translation2d moduleTranslationOnRobot,
            int tickNum,
            double tickPeriodSeconds
    ) {
        /* update the DC motor simulation of the steer */
        module.updateSim(tickPeriodSeconds);

        /* simulate the propelling force of the module */
        final Rotation2d moduleWorldFacing = module.getSimulationSteerFacing().plus(robotWorldPose.getRotation());
        final Vector2 moduleWorldPosition = GeometryConvertor.toDyn4jVector2(
                robotWorldPose.getTranslation()
                        .plus(moduleTranslationOnRobot.rotateBy(robotWorldPose.getRotation()))
        );
        double actualPropellingForceOnFloorNewtons = module.getSimulationTorque() / WHEEL_RADIUS_METERS;
        final boolean skidding = Math.abs(actualPropellingForceOnFloorNewtons) > MAX_FRICTION_FORCE_PER_MODULE;
        if (skidding)
            actualPropellingForceOnFloorNewtons = Math.copySign(MAX_FRICTION_FORCE_PER_MODULE, actualPropellingForceOnFloorNewtons);
        super.applyForce(
                Vector2.create(actualPropellingForceOnFloorNewtons, moduleWorldFacing.getRadians()),
                moduleWorldPosition
        );


        final Vector2 floorVelocity = super.getLinearVelocity(moduleWorldPosition);
        final double floorVelocityProjectionOnWheelDirectionMPS = floorVelocity.getMagnitude() *
                Math.cos(floorVelocity.getAngleBetween(new Vector2(moduleWorldFacing.getRadians())));

        if (skidding)
            /* if the chassis is skidding, the toque will cause the wheels to spin freely */
            module.physicsSimulationResults.driveWheelFinalVelocityRadPerSec += module.getSimulationTorque() / DRIVE_INERTIA * tickPeriodSeconds;
        else
            /* otherwise, the floor velocity is projected to the wheel */
            module.physicsSimulationResults.driveWheelFinalVelocityRadPerSec = floorVelocityProjectionOnWheelDirectionMPS / WHEEL_RADIUS_METERS;

        module.physicsSimulationResults.odometrySteerPositions[tickNum] = module.getSimulationSteerFacing();
        module.physicsSimulationResults.driveWheelFinalRevolutions += Units.radiansToRotations(
                module.physicsSimulationResults.driveWheelFinalVelocityRadPerSec * tickPeriodSeconds
        );
        module.physicsSimulationResults.odometryDriveWheelRevolutions[tickNum] = module.physicsSimulationResults.driveWheelFinalRevolutions;
    }

    private void simulateFrictionForce() {
        final ChassisSpeeds speedsDifference = getDifferenceBetweenFloorAndFreeSpeed();
        final Translation2d translationalSpeedsDifference = new Translation2d(speedsDifference.vxMetersPerSecond, speedsDifference.vyMetersPerSecond);
        final double forceMultiplier = Math.min(translationalSpeedsDifference.getNorm() * 3, 1);
        super.applyForce(Vector2.create(
                forceMultiplier * MAX_FRICTION_ACCELERATION * ROBOT_MASS_KG,
                translationalSpeedsDifference.getAngle().getRadians()
        ));

        if (Math.abs(getDesiredSpeedsFieldRelative().omegaRadiansPerSecond)
                / CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC
                < 0.01)
            simulateChassisRotationalBehavior(0);
    }

    private ChassisSpeeds getDifferenceBetweenFloorAndFreeSpeed() {
        ChassisSpeeds chassisFreeSpeedsFieldRelative = getFreeSpeedsFieldRelative();

        final double freeSpeedMagnitude = Math.hypot(chassisFreeSpeedsFieldRelative.vxMetersPerSecond, chassisFreeSpeedsFieldRelative.vyMetersPerSecond),
                floorSpeedMagnitude = Math.hypot(getMeasuredChassisSpeedsFieldRelative().vxMetersPerSecond, getMeasuredChassisSpeedsFieldRelative().vyMetersPerSecond);
        if (freeSpeedMagnitude > floorSpeedMagnitude)
            chassisFreeSpeedsFieldRelative = chassisFreeSpeedsFieldRelative.times(floorSpeedMagnitude / freeSpeedMagnitude);

        return chassisFreeSpeedsFieldRelative.minus(getMeasuredChassisSpeedsFieldRelative());
    }

    private ChassisSpeeds getFreeSpeedsFieldRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                DRIVE_KINEMATICS.toChassisSpeeds(
                        Arrays.stream(modules)
                                .map(ModuleIOSim::getSimulationSwerveState)
                                .toArray(SwerveModuleState[]::new)),
                getObjectOnFieldPose2d().getRotation()
        );
    }

    private ChassisSpeeds getDesiredSpeedsFieldRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                DRIVE_KINEMATICS.toChassisSpeeds(
                        Arrays.stream(modules)
                                .map(ModuleIOSim::getDesiredSwerveState)
                                .toArray(SwerveModuleState[]::new)),
                getObjectOnFieldPose2d().getRotation()
        );
    }

    private void gyroSimulationSubTick(
            Rotation2d currentFacing,
            double angularVelocityRadPerSec,
            int tickNum
    ) {
        final GyroIOSim.GyroPhysicsSimulationResults results = gyroIOSim.gyroPhysicsSimulationResults;
        results.robotAngularVelocityRadPerSec = angularVelocityRadPerSec;
        results.odometryYawPositions[tickNum] = currentFacing;
        results.hasReading = true;
    }

    public static final class OdometryThreadSim implements OdometryThread {
        @Override
        public void updateInputs(OdometryThreadInputs inputs) {
            inputs.measurementTimeStamps = new double[SIMULATION_TICKS_IN_1_PERIOD];
            final double robotStartingTimeStamps = MapleTimeUtils.getLogTimeSeconds(),
                    iterationPeriodSeconds = Robot.defaultPeriodSecs/SIMULATION_TICKS_IN_1_PERIOD;
            for (int i =0; i < SIMULATION_TICKS_IN_1_PERIOD; i++)
                inputs.measurementTimeStamps[i] = robotStartingTimeStamps + i * iterationPeriodSeconds;
        }
    }
}
