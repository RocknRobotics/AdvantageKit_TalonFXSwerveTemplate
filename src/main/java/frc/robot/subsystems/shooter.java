package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {
  public double acceleration = .1;
  public SparkFlex shootingMotor = new SparkFlex(2, MotorType.kBrushless);
  public SparkFlex loadingMotor = new SparkFlex(3, MotorType.kBrushless);
  public PS4Controller fuelManipulater = new PS4Controller(1);

  @SuppressWarnings("removal")
  public void update() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.openLoopRampRate(acceleration);
    config.closedLoopRampRate(acceleration);
    shootingMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    loadingMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void increase() {
    if (acceleration < 1) {
      acceleration += .01;
      update();
    }
  }

  public void decrease() {
    if (acceleration > 0) {
      acceleration -= .01;
      update();
    }
  }

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

  public double getShooterSpeed() {
    return shootingMotor.get();
  }

  public void periodic() {
    if (fuelManipulater.getSquareButtonPressed()) {
      shoot(.15);
    }

    // Picks up ball from fuel tank at speed of right motor
    if (fuelManipulater.getCrossButtonPressed()) {
      load(.5);
    }

    if (fuelManipulater.getR1ButtonPressed()) {
      load(0);
      shoot(0);
    }
  }
}
