package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class arm extends SubsystemBase {
  public SparkMax arm = new SparkMax(1, MotorType.kBrushless);
  public PS4Controller ps4 = new PS4Controller(1);
  public RelativeEncoder encoder = arm.getEncoder();

  public arm() {
    setup();
  }

  public void setup() {
    encoder.setPosition(0);
  }

  public void end() {
    while (encoder.getPosition() < 90) {
      arm.set(.1);
    }
    arm.set(0);
  }

  public void start() {
    while (encoder.getPosition() > 0) {
      arm.set(.1);
    }
    arm.set(0);
  }

  public void periodic() {
    if (ps4.getCircleButtonPressed()) {
      start();
    }
    if (ps4.getTriangleButtonPressed()) {
      end();
    }
  }
}
