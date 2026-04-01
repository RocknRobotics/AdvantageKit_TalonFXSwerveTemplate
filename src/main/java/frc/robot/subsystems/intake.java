package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class intake extends SubsystemBase {
  // public double targetY = LimelightHelpers.getTY("limelight");
  // public double targetX = LimelightHelpers.getTX("limelight");

  // private PS4Controller fuelMinipulater = new PS4Controller(1);
  public double posi = 0;
  public double kg = .5;
  public double posp = 0;
  public double posd = 0;
  static final double COUNTS_PER_MOTOR_REV = 288;
  static final double GEAR_REDUCTION = 2.7778;
  static final double COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
  static final double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV / 360;
  // Postion Conversion Factor: 360 / GearRatio
  private final double PCF = 360.0 / 108.0;
  // Velocity Conversion Factor
  private final double VCF = PCF / 60.0;
  public double acceleration = .1;
  public SparkFlex intakeleft = new SparkFlex(18, MotorType.kBrushless);
  // public SparkFlex intakeright = new SparkFlex(16, MotorType.kBrushless);
  public SparkFlex roller = new SparkFlex(17, MotorType.kBrushless);
  private RelativeEncoder intakeArmEncoder = intakeleft.getEncoder();

  public SparkClosedLoopController left;
  // public SparkClosedLoopController right;
  public double speedin = .30;

  // private double targetPosition = 60.0;

  public intake() {
    configmotors();
  }

  @SuppressWarnings("removal")
  public void configmotors() {
    // SparkFlexConfig configright = new SparkFlexConfig();
    SparkFlexConfig configleft = new SparkFlexConfig();
    SparkFlexConfig configroller = new SparkFlexConfig();
    configleft
        .openLoopRampRate(.3)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .inverted(false)
        .closedLoopRampRate(.3)
        .closedLoop
        .p(posp)
        .i(posi)
        .d(posd)
        .outputRange(-1, 1)
        .feedForward
        .kG(kg);
    configleft.encoder.positionConversionFactor(PCF).velocityConversionFactor(VCF);
    // configright
    //     .openLoopRampRate(.3)
    //     .idleMode(IdleMode.kBrake)
    //     .inverted(false)
    //     .smartCurrentLimit(40)
    //     .closedLoopRampRate(.3)
    //     .closedLoop
    //     .p(posp)
    //     .i(posi)
    //     .d(posd)
    //     .outputRange(-1, 1)
    //     .feedForward
    //     .kG(kg);
    configroller
        .openLoopRampRate(.3)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(80)
        .closedLoopRampRate(.3);
    // right = intakeright.getClosedLoopController();
    // intakeright.configure(
    //     configright, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeleft.configure(
        configleft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    roller.configure(configroller, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    left = intakeleft.getClosedLoopController();
    // SmartDashboard.putNumber(null, acceleration);
  }

  public void updateIntakeSpeed() {
    // roller.set(speedin);
    // SmartDashboard.putNumber(null, acceleration);
  }

  // public void increaseIntakeSpeed() {
  //   if (speedin < 1) {
  //     speedin += .01;
  //   }
  //   if (speedin < 0) {
  //     speedin = 0.01;
  //   }
  //   updateIntakeSpeed();
  // }

  // public void decreaseIntakeSpeed() {
  //   if (speedin > 0) {
  //     speedin -= .01;
  //   }
  //   if (speedin < 0) {
  //     speedin = 0;
  //   }
  //   updateIntakeSpeed();
  // }
  /**
   * Moves the Intake up or down to a specified position
   *
   * @param position Target Position
   */
  // look at tommarow
  // public Command Up()
  // {
  //  return new RunCommand(()->intakeleft.set(.4));
  // }
  public Command setIntakePosition(double position) {
    return new RunCommand(() -> left.setSetpoint(position, ControlType.kPosition));
  }

  /**
   * Sets the Intake raise/lower speed
   *
   * @param speed Target Speed
   */
  public Command setIntakeSpeed(double speed) {
    // if (intakeArmEncoder.getPosition() > 0 && speed > 0) {
    //   return new RunCommand(() -> intakeleft.set(speed));
    // } else if (intakeArmEncoder.getPosition() < 30 && speed < 0) {
    //   return new RunCommand(() -> intakeleft.set(speed));
    // } else {
    //   return new RunCommand(() -> intakeleft.stopMotor());
    // }
    return new RunCommand(() -> intakeleft.set(speed));
  }

  public Command setIntakeStop() {
    return new RunCommand(() -> intakeleft.stopMotor());
  }

  /**
   * Sets the Rollar speed to a specified amount
   *
   * @param speed Target Speed
   */
  public Command setSpeed(double speed) {
    return new RunCommand(() -> roller.set(speed));
  }

  public Command setRollerStop() {
    return new RunCommand(() -> roller.stopMotor());
  }

  // public void changePosition() {
  //   targetPosition = targetPosition == 60.0 ? 0.0 : 60.0;
  // }

  @Override
  public void periodic() {
    // left.setSetpoint(targetPosition, ControlType.kPosition);
    // if (intakeleft.getEncoder().getPosition() <= 0 || intakeleft.getEncoder().getPosition() >=
    // 60)
    //   setIntakeStop();
    SmartDashboard.putNumber("Intake Pivot value", intakeleft.getEncoder().getPosition());
    // intakeleft.set(fuelMinipulater.getLeftY() / 3.0);
  }
}
