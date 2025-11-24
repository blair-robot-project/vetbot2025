package frc.team449.subsystems.pivot

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.system.motor.createKraken
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command

@Logged
class Pivot (
  val motor: TalonFX,
) : SubsystemBase() {

  fun setPosition(position: Angle): Command {
    return runOnce {
      motor.setPosition(position.`in`(Rotations))
    }
  }

  fun stop(): Command {
    return runOnce {
      motor.stopMotor()
    }
  }

  fun coast(): Command {
    return runOnce {
      val newConfig = TalonFXConfiguration()
      newConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast
      motor.configurator.apply(newConfig)
    }
  }

  fun brake(): Command {
    return runOnce {
      val newConfig = TalonFXConfiguration()
      newConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
      motor.configurator.apply(newConfig)
    }
  }

  companion object {
    fun createPivot() : Pivot {
      val pivotMotor = createKraken(
        PivotConstants.PIVOT_ID,
        PivotConstants.PIVOT_INVERTED,
        kS = PivotConstants.PIVOT_KS,
        kV = PivotConstants.PIVOT_KV,
        kA = PivotConstants.PIVOT_KA,
        kG = PivotConstants.PIVOT_KG,
        kP = PivotConstants.PIVOT_KP,
        kI = PivotConstants.PIVOT_KI,
        kD = PivotConstants.PIVOT_KD,
        gravityType = GravityTypeValue.Arm_Cosine,
        cruiseVel = PivotConstants.PIVOT_CRUISE_VEL.`in`(RotationsPerSecond),
        maxAccel = PivotConstants.PIVOT_MAX_ACCEL.`in`(RotationsPerSecondPerSecond)
      )
      return Pivot(pivotMotor)
    }
  }
}
