package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.TurretConstants.TurretConfig;
import java.util.function.Supplier;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretConfig config;
  private final Supplier<Pose2d> poseSupplier;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private boolean trackingEnabled = true;
  private Rotation2d setpoint = Rotation2d.kZero;

  public Turret(TurretIO io, TurretConfig config, Supplier<Pose2d> poseSupplier) {
    this.io = io;
    this.config = config;
    this.poseSupplier = poseSupplier;
    setName("Turret/" + config.name());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (trackingEnabled) {
      setpoint = calculateSetpoint(poseSupplier.get());
      io.setPosition(setpoint);
    }
  }

  // Track the Hub based on field pos
  private Rotation2d calculateSetpoint(Pose2d robotPose) {
    Translation2d turretFieldPosition =
        robotPose.getTranslation().plus(config.robotToTurret().rotateBy(robotPose.getRotation()));
    Rotation2d fieldRelativeBearing =
        TurretConstants.HUB_LOCATION.minus(turretFieldPosition).getAngle();
    Rotation2d turretRelativeSetpoint =
        fieldRelativeBearing.minus(robotPose.getRotation()).minus(config.mountingOffset());

    double clampedRadians =
        MathUtil.clamp(
            turretRelativeSetpoint.getRadians(),
            TurretConstants.MIN_ANGLE.getRadians(),
            TurretConstants.MAX_ANGLE.getRadians());
    return Rotation2d.fromRadians(clampedRadians);
  }

  public boolean atGoal() {
    return Math.abs(inputs.position.minus(setpoint).getRadians())
        < TurretConstants.GOAL_TOLERANCE_RADIANS;
  }

  public Rotation2d getPosition() {
    return inputs.position;
  }
}
