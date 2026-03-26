package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {
  public double acceleration = .1;
  public double shooterAccel = 0.5;
  public SparkFlex shootingMotor = new SparkFlex(19, MotorType.kBrushless);
  public SparkFlex loadingMotor = new SparkFlex(20, MotorType.kBrushless);
  public SparkMax convayerbelt = new SparkMax(21, MotorType.kBrushless);

  @SuppressWarnings("removal")
  public void update() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.openLoopRampRate(acceleration);
    config.closedLoopRampRate(acceleration);
    loadingMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.openLoopRampRate(shooterAccel);
    config.closedLoopRampRate(shooterAccel);
    shootingMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // public void increase() {
  //   if (acceleration < 1) {
  //     acceleration += .01;
  //     update();
  //   }
  // }

  // public void decrease() {
  //   if (acceleration > 0) {
  //     acceleration -= .01;
  //     update();
  //   }
  // }

  public void load(double speed) {
    loadingMotor.set(speed);
  }

  public void shoot(double power) {
    if (power < -1) {
      power = -1;
    }
    if (power > 1) {
      power = 1;
    }

    shootingMotor.set(power);
  }

  public Command setShooterSpeed(double speed) {
    return new RunCommand(() -> shootingMotor.set(speed));
  }

  public Command setTransition(double speed) {
    return new RunCommand(() -> loadingMotor.set(speed));
  }

  public Command setConvayerbelt(double speed) {
    return new RunCommand(() -> convayerbelt.set(-speed));
  }

  public double getShooterSpeed() {
    return shootingMotor.get();
  }

  // public void periodic() {}
}
