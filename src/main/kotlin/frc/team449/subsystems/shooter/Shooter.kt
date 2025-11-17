package frc.team449.subsystems.shooter

import au.grapplerobotics.LaserCan
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.revrobotics.spark.SparkMax
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Velocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.createFollowerSpark
import frc.team449.system.motor.createKraken
import frc.team449.system.motor.createSparkMax

class Shooter (
  val intakeLeader: SparkMax,
  val intakeFollower: SparkMax,
  val shooterMotor: TalonFX,
  val sensor: LaserCan
): SubsystemBase() {

  fun setShooterVelocity(velocity: AngularVelocity): Command {
    return runOnce {
      shooterMotor.setControl(MotionMagicVelocityVoltage(velocity))
    }
  }

  fun intake(): Command {
    return runOnce {
      intakeLeader.setVoltage(ShooterConstants.INTAKE_VOLTAGE)
    }
  }

  fun outtake(): Command {
    return runOnce {
      intakeLeader.setVoltage(ShooterConstants.OUTTAKE_VOLTAGE)
    }
  }

  fun stopShooter(): Command {
    return runOnce {
      shooterMotor.stopMotor()
    }
  }

  fun stopIntaker(): Command {
    return runOnce {
      intakeLeader.stopMotor()
    }
  }

  fun stop(): Command {
    return runOnce {
      shooterMotor.stopMotor()
      intakeLeader.stopMotor()
    }
  }

  fun hasPiece(): Boolean {
    return sensor.measurement.distance_mm < ShooterConstants.DETECTION_TARGET_MM
  }

  companion object {
    fun createShooter(): Shooter {
      val intakeLeader = createSparkMax(
        ShooterConstants.INTAKE_LEADER_ID,
        ShooterConstants.INTAKE_LEADER_INVERTED
      )
      val intakeFollower = createFollowerSpark(
        ShooterConstants.INTAKE_FOLLOWER_ID,
        intakeLeader,
        ShooterConstants.INTAKE_FOLLOWER_INVERTED
      )
      val shooterMotor = createKraken(
        ShooterConstants.SHOOTER_ID,
        ShooterConstants.SHOOTER_INVERTED,
        brakeMode = false,
        kS = ShooterConstants.SHOOTER_KS,
        kV = ShooterConstants.SHOOTER_KV,
        kA = ShooterConstants.SHOOTER_KA,
        kP = ShooterConstants.SHOOTER_KP,
        kI = ShooterConstants.SHOOTER_KI,
        kD = ShooterConstants.SHOOTER_KD,
      )
      val laserCan = LaserCan(
        ShooterConstants.LASER_CAN_ID
      )
      return Shooter(
        intakeLeader,
        intakeFollower,
        shooterMotor,
        laserCan
      )
    }
  }
}