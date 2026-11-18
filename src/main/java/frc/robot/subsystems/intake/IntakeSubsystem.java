package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax roller;
  private final SparkMax pivot;
  private final PIDController pid;
  private final SparkAbsoluteEncoder pivotEncoder;

  private Rotation2d target;

  private boolean piece; // true is cones
  private boolean hasPiece = false;

  private final Timer intakeTimer = new Timer();
  private final Debouncer intakeDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

  public IntakeSubsystem() {
    pid = new PIDController(IntakeConstants.kp, IntakeConstants.ki, IntakeConstants.kd);

    pid.setTolerance(IntakeConstants.pidTolerance.getDegrees());
    pid.setIZone(IntakeConstants.pidIzone.getDegrees());

    pivot = new SparkMax(IntakeConstants.pivotID, SparkLowLevel.MotorType.kBrushless);
    roller = new SparkMax(IntakeConstants.rollerID, SparkLowLevel.MotorType.kBrushless);
    pivotEncoder = pivot.getAbsoluteEncoder();

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.pivotCurrentLim)
        .voltageCompensation(IntakeConstants.pivotVoltageCompensation);

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig
        .idleMode(SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.rollerCurrentLim)
        .voltageCompensation(IntakeConstants.rollerVoltageCompensation);

    roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    target = getAngle();
  }

  private double gravityFactor(Rotation2d angle) {
    return Math.cos(angle.getRadians()) * IntakeConstants.kG;
  }

  public Rotation2d getAngle() {
    double deg = (Rotation2d.fromRotations(pivotEncoder.getPosition()).getDegrees());
    return Rotation2d.fromDegrees(MathUtil.inputModulus(deg, -90, 270));
  }

  private void setAngle(Rotation2d angle) {
    target =
        Rotation2d.fromDegrees(
            MathUtil.clamp(
                angle.getDegrees(), IntakeConstants.minAngleDeg, IntakeConstants.maxAngleDeg));
  }

  public void resetPID() {
    pid.reset();
    target = getAngle();
  }

  public void roll(double speed) {
    roller.set(speed);
  }

  public void setPivot(boolean state) {
    if (state) {
      setAngle(IntakeConstants.activeAngle);
    } else {
      setAngle(IntakeConstants.stowAngle);
    }
  }

  public void rollIn() { // intake the bag things or spit the cones
    roll(IntakeConstants.intakeSpeed);
  }

  public void rollOut() { // intake the cones or spit the bag things
    roll(-1 * (IntakeConstants.intakeSpeed));
  }

  public void stopRollers() {
    roll(0);
  }

  public void setPieceState(boolean target) {
    piece = target;
  }

  public boolean isActive() {
    return Math.abs(getAngle().getDegrees() - IntakeConstants.activeAngle.getDegrees())
        < IntakeConstants.activeTolerance;
  }

  public boolean isStowed() {
    return Math.abs(getAngle().getDegrees() - IntakeConstants.stowAngle.getDegrees())
        < IntakeConstants.activeTolerance;
  }

  public boolean statorCheck() {
    return roller.getOutputCurrent() > IntakeConstants.holdingStatorSpike;
  }

  public Command stow() {
    return Commands.sequence(
            runOnce(
                () -> {
                  stopRollers();
                  setPivot(false);
                }),
            Commands.waitUntil(this::isStowed))
        .withTimeout(2.0);
  }

  private Command intake(Runnable rollerAction, boolean piece) {
    return this.startEnd(
            () -> {
              setPivot(true);
              rollerAction.run();
              intakeTimer.restart();
              intakeDebouncer.calculate(false);
            },
            () -> {
              stopRollers();
              setPivot(false);
            })
        .until(() -> intakeTimer.hasElapsed(0.25) && intakeDebouncer.calculate(statorCheck()))
        .andThen(
            runOnce(
                () -> {
                  hasPiece = true;
                  setPieceState(piece);
                }))
        .unless(() -> hasPiece);
  }

  public Command pieceReset() {
    return Commands.runOnce(() -> hasPiece = false);
  }

  public Command intakeCones() {
    return intake(this::rollOut, true);
  }

  public Command intakeCubes() {
    return intake(this::rollIn, false);
  }

  private Command spit(Runnable rollerAction) {
    return Commands.sequence(
            runOnce(() -> setPivot(true)), Commands.waitUntil(this::isActive), run(rollerAction))
        .finallyDo(
            () -> {
              stopRollers();
              setPivot(false);
              hasPiece = false;
            });
  }

  public Command outtake() {
    return Commands.either(spit(this::rollIn), spit(this::rollOut), () -> piece);
  }

  @Override
  public void periodic() {

    double targetAngle = target.getDegrees();
    double angleDeg = getAngle().getDegrees();
    double pidOut = pid.calculate(angleDeg, targetAngle);
    double gravity =
        gravityFactor(Rotation2d.fromDegrees(angleDeg + IntakeConstants.angleFromZero));
    double finalOut = MathUtil.clamp((pidOut + gravity), -0.7, 0.7);
    pivot.set(finalOut);

    SmartDashboard.putNumber("Intake/AngleDeg", angleDeg);
    SmartDashboard.putNumber("Intake/TargetDeg", targetAngle);
    SmartDashboard.putNumber("Intake/PivotOutput", finalOut);
    SmartDashboard.putNumber("Intake/RollerCurrent", roller.getOutputCurrent());
    SmartDashboard.putBoolean("Intake/AtSetpoint", pid.atSetpoint());
    SmartDashboard.putBoolean("Intake/IsActive", isActive());
    SmartDashboard.putBoolean("Intake/HasPiece", hasPiece);
    SmartDashboard.putString("Intake/PieceType", !hasPiece ? "None" : (piece ? "Cone" : "Cube"));
  }
}
