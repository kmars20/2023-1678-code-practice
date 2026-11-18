package frc.robot.subsystems.elevator;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax lengthMotor, oppositeLength, pivot, oppositePivot;
  private final PIDController pivotPid, lengthPid;

  private PState pivotTarget = PState.Stow;
  private LState lengthTarget = LState.L1;
  private boolean wasDisabled = true;
  private boolean climbed = false;

  public enum LState {
    L1(ElevatorConstants.L1Length),
    L2(ElevatorConstants.L2Length),
    L3(ElevatorConstants.L3Length);

    public final double length;

    LState(double length) {
      this.length = length;
    }

    public double getTargetLength() {
      return length;
    }
  }

  public enum PState {
    Stow(ElevatorConstants.stowAngle),
    L1(ElevatorConstants.L1Angle),
    L2(ElevatorConstants.L2Angle),
    L3(ElevatorConstants.L3Angle);

    public final Rotation2d angle;

    PState(Rotation2d angle) {
      this.angle = angle;
    }

    public Rotation2d getTargetAngle() {
      return angle;
    }
  }

  public ElevatorSubsystem() {
    pivotPid = new PIDController(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd);

    pivotPid.setTolerance(ElevatorConstants.pidTolerance.getDegrees());
    pivotPid.setIZone(ElevatorConstants.pidIzone.getDegrees());

    lengthPid =
        new PIDController(ElevatorConstants.Lkp, ElevatorConstants.Lki, ElevatorConstants.Lkd);

    lengthPid.setTolerance(ElevatorConstants.LpidTolerance);
    lengthPid.setIZone(ElevatorConstants.LpidIzone);

    pivot = new SparkMax(ElevatorConstants.pivotID, SparkLowLevel.MotorType.kBrushless);
    oppositePivot =
        new SparkMax(ElevatorConstants.oppositePivotID, SparkLowLevel.MotorType.kBrushless);
    lengthMotor = new SparkMax(ElevatorConstants.lengthID, SparkLowLevel.MotorType.kBrushless);
    oppositeLength =
        new SparkMax(ElevatorConstants.oppositeLengthID, SparkLowLevel.MotorType.kBrushless);

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.pivotCurrentLim)
        .voltageCompensation(ElevatorConstants.pivotVoltageCompensation);
    pivotConfig.encoder.positionConversionFactor(360.0 / ElevatorConstants.pivotRatio);

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig oppositePivotConfig = new SparkMaxConfig();
    oppositePivotConfig
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.pivotCurrentLim)
        .voltageCompensation(ElevatorConstants.pivotVoltageCompensation)
        .follow(pivot, true);

    oppositePivot.configure(
        oppositePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig lengthConfig = new SparkMaxConfig();
    lengthConfig
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.lengthCurrentLim)
        .voltageCompensation(ElevatorConstants.lengthVoltageCompensation);
    lengthConfig.encoder.positionConversionFactor(
        (Math.PI * ElevatorConstants.spoolDiam) / ElevatorConstants.spoolRatio);

    lengthMotor.configure(
        lengthConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig oppositeLengthConfig = new SparkMaxConfig();
    oppositeLengthConfig
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.lengthCurrentLim)
        .voltageCompensation(ElevatorConstants.lengthVoltageCompensation)
        .follow(lengthMotor, true);

    oppositeLength.configure(
        oppositeLengthConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(pivot.getEncoder().getPosition());
  }

  public double getLength() {
    return lengthMotor.getEncoder().getPosition();
  }

  public boolean pivotAtGoal() {
    return pivotPid.atSetpoint();
  }

  public boolean lengthAtGoal() {
    return lengthPid.atSetpoint();
  }

  public boolean isClimbed() {
    return climbed;
  }

  public boolean isStowed() {
    boolean pivotStowed =
        Math.abs(getAngle().getDegrees() - PState.Stow.getTargetAngle().getDegrees())
            <= ElevatorConstants.pidTolerance.getDegrees();
    boolean lengthStowed =
        Math.abs(getLength() - LState.L1.getTargetLength()) <= ElevatorConstants.LpidTolerance;
    return pivotStowed && lengthStowed;
  }

  private void setAngle(PState state) {
    pivotTarget = state;
  }

  private void setLength(LState state) {
    lengthTarget = state;
  }

  public void resetPID() {
    pivotPid.reset();
    lengthPid.reset();
  }

  private double pivotGravity(Rotation2d angleFromHorizontal, double length) {
    return Math.cos(angleFromHorizontal.getRadians())
        * (ElevatorConstants.PkG + ElevatorConstants.PkGPerLength * length);
  }

  private double lengthGravity(Rotation2d angleFromHorizontal) {
    return Math.sin(angleFromHorizontal.getRadians()) * ElevatorConstants.LkG;
  }

  private Command goTo(PState p, LState l) {
    return Commands.sequence(
            Commands.runOnce(() -> setLength(LState.L1), this),
            Commands.waitUntil(this::lengthAtGoal),
            Commands.runOnce(() -> setAngle(p), this),
            Commands.waitUntil(this::pivotAtGoal),
            Commands.runOnce(() -> setLength(l), this),
            Commands.waitUntil(this::lengthAtGoal))
        .withTimeout(4.0)
        .unless(this::isClimbed);
  }

  public Command L1() {
    return goTo(PState.L1, LState.L1);
  }

  public Command L2() {
    return goTo(PState.L2, LState.L2);
  }

  public Command L3() {
    return goTo(PState.L3, LState.L3);
  }

  public Command Stow() {
    return goTo(PState.Stow, LState.L1);
  }

  public Command lock() {
    return Commands.runOnce(() -> climbed = true, this);
  }

  public Command unclimb() {
    return Commands.runOnce(
            () -> {
              climbed = false;
              wasDisabled = true;
            },
            this)
        .andThen(Stow());
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      wasDisabled = true;
      pivot.set(0);
      lengthMotor.set(0);
      return;
    }

    if (wasDisabled) {
      resetPID();
      wasDisabled = false;
    }

    if (climbed) {
      pivot.set(0);
      lengthMotor.set(0);
      return;
    }

    double angleDeg = getAngle().getDegrees();
    double length = getLength();
    Rotation2d fromHorizontal = getAngle().plus(ElevatorConstants.angleFromZero);

    double pivotSetpoint =
        MathUtil.clamp(
            pivotTarget.getTargetAngle().getDegrees(),
            ElevatorConstants.pivotMinDeg,
            ElevatorConstants.pivotMaxDeg);
    double lengthSetpoint =
        MathUtil.clamp(
            lengthTarget.getTargetLength(),
            ElevatorConstants.lengthMin,
            ElevatorConstants.lengthMax);

    double pivotOut =
        pivotPid.calculate(angleDeg, pivotSetpoint) + pivotGravity(fromHorizontal, length);
    double lengthOut = lengthPid.calculate(length, lengthSetpoint) + lengthGravity(fromHorizontal);

    if (angleDeg <= ElevatorConstants.pivotMinDeg && pivotOut < 0) pivotOut = 0;
    if (angleDeg >= ElevatorConstants.pivotMaxDeg && pivotOut > 0) pivotOut = 0;
    if (length <= ElevatorConstants.lengthMin && lengthOut < 0) lengthOut = 0;
    if (length >= ElevatorConstants.lengthMax && lengthOut > 0) lengthOut = 0;

    pivot.set(MathUtil.clamp(pivotOut, -0.7, 0.7));
    lengthMotor.set(MathUtil.clamp(lengthOut, -0.7, 0.7));

    SmartDashboard.putNumber("Elevator/AngleDeg", angleDeg);
    SmartDashboard.putNumber("Elevator/Length", length);
    SmartDashboard.putString("Elevator/PivotTarget", pivotTarget.name());
    SmartDashboard.putString("Elevator/LengthTarget", lengthTarget.name());
  }
}
