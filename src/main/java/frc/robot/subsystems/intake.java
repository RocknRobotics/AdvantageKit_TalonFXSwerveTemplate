package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class intake extends SubsystemBase {
  // public double targetY = LimelightHelpers.getTY("limelight");
  // public double targetX = LimelightHelpers.getTX("limelight");
  public double acceleration = .1;
  public SparkFlex updownmotor1 = new SparkFlex(4, MotorType.kBrushless);
  public SparkFlex updownmotor2 = new SparkFlex(5, MotorType.kBrushless);
  public SparkFlex pickupmotor = new SparkFlex(6, MotorType.kBrushless);
  public double speedin = .30;
  public CommandPS4Controller p4 = new CommandPS4Controller(1);

  @SuppressWarnings("removal")
  public void updateAccleration() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.openLoopRampRate(acceleration);
    config.closedLoopRampRate(acceleration);
    updownmotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    updownmotor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pickupmotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // SmartDashboard.putNumber(null, acceleration);
  }

  public void updateIntakeSpeed() {
    pickupmotor.set(speedin);
    // SmartDashboard.putNumber(null, acceleration);
  }

  public void increaseAcceleration() {
    if (acceleration < 1) {
      acceleration += .01;
    }
    if (acceleration < 0) {
      acceleration = 0.01;
    }
    updateAccleration();
  }

  public void decreaseAcceleration() {
    if (acceleration > 0) {
      acceleration -= .01;
    }
    if (acceleration < 0) {
      acceleration = 0;
    }
    updateAccleration();
  }

  public void increaseIntakeSpeed() {
    if (speedin < 1) {
      speedin += .01;
    }
    if (speedin < 0) {
      speedin = 0.01;
    }
    updateIntakeSpeed();
  }

  public void decreaseIntakeSpeed() {
    if (speedin > 0) {
      speedin -= .01;
    }
    if (speedin < 0) {
      speedin = 0;
    }
    updateIntakeSpeed();
  }

  public void setDirection(double speed) {
    updownmotor1.set(speed);
    updownmotor2.set(-speed);
  }

  public void periodic() {
    setDirection(p4.getLeftY());
    pickupmotor.set(0.60);
  }
}
