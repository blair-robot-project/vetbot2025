package frc.team449.subsystems.intake_pivot

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.spark.SparkMax
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.system.motor.createFollowerSpark
import frc.team449.system.motor.createKraken
import frc.team449.system.motor.createSparkMax
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command

@Logged
class IntakePivot (
  val pivot_motor: TalonFX,
) : SubsystemBase() {

  fun setPosition(position: Angle): Command {
    return runOnce {
      pivot_motor.setPosition(position.`in`(Rotations))
    }
  }

  fun stop(): Command {
    return runOnce {
      pivot_motor.stopMotor()
    }
  }

  fun coast(): Command {
    return runOnce {
      val new_config = TalonFXConfiguration()
      new_config.MotorOutput.NeutralMode = NeutralModeValue.Coast
      pivot_motor.configurator.apply(new_config)
    }
  }

  fun brake(): Command {
    return runOnce {
      val new_config = TalonFXConfiguration()
      new_config.MotorOutput.NeutralMode = NeutralModeValue.Brake
      pivot_motor.configurator.apply(new_config)
    }
  }

  companion object {
    fun createIntakePivot() {
      val pivot_motor = createKraken(
        IntakePivotConstants.PIVOT_ID,
        IntakePivotConstants.PIVOT_INVERTED,
        kS = IntakePivotConstants.PIVOT_KS,
        kV = IntakePivotConstants.PIVOT_KV,
        kA = IntakePivotConstants.PIVOT_KA,
        kG = IntakePivotConstants.PIVOT_KG,
        kP = IntakePivotConstants.PIVOT_KP,
        kI = IntakePivotConstants.PIVOT_KI,
        kD = IntakePivotConstants.PIVOT_KD,
        gravityType = GravityTypeValue.Arm_Cosine,
        cruiseVel = IntakePivotConstants.PIVOT_CRUISE_VEL.`in`(RotationsPerSecond),
        maxAccel = IntakePivotConstants.PIVOT_MAX_ACCEL.`in`(RotationsPerSecondPerSecond)
      )
    }
  }
}
