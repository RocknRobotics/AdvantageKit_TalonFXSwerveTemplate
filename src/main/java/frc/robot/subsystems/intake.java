package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class intake extends SubsystemBase {
  // public double targetY = LimelightHelpers.getTY("limelight");
  // public double targetX = LimelightHelpers.getTX("limelight");

  private PS4Controller fuelMinipulater = new PS4Controller(1);
  public double posi = 0;
  public double kg = .5;
  public double posp = 0;
  public double posd = 0;
  public double acceleration = .1;
  public SparkFlex intakeleft = new SparkFlex(18, MotorType.kBrushless);
  // public SparkFlex intakeright = new SparkFlex(16, MotorType.kBrushless);
  public SparkFlex roller = new SparkFlex(17, MotorType.kBrushless);
  public SparkClosedLoopController left;
  // public SparkClosedLoopController right;
  public double speedin = .30;

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
    left = intakeleft.getClosedLoopController();
    // right = intakeright.getClosedLoopController();
    // intakeright.configure(
    //     configright, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeleft.configure(
        configleft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    roller.configure(configroller, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // SmartDashboard.putNumber(null, acceleration);
  }

  public void updateIntakeSpeed() {
    roller.set(speedin);
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

  public Command setIntakePosition(double position) {
    return new RunCommand(() -> left.setSetpoint(position, ControlType.kPosition));
  }

  public Command setIntakeSpeed(double speed) {
    return new RunCommand(() -> intakeleft.set(speed));
  }

  public Command setSpeed(double speed) {
    return new RunCommand(() -> roller.set(speed));
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Intake Pivot value", intakeleft.getEncoder().getPosition());
    intakeleft.set(fuelMinipulater.getLeftY() / 3.0);
  }
}
