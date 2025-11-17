package frc.team449.subsystems.intake_pivot

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
class IntakePivot (
  val pivotMotor: TalonFX,
) : SubsystemBase() {

  fun setPosition(position: Angle): Command {
    return runOnce {
      pivotMotor.setPosition(position.`in`(Rotations))
    }
  }

  fun stop(): Command {
    return runOnce {
      pivotMotor.stopMotor()
    }
  }

  fun coast(): Command {
    return runOnce {
      val newConfig = TalonFXConfiguration()
      newConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast
      pivotMotor.configurator.apply(newConfig)
    }
  }

  fun brake(): Command {
    return runOnce {
      val newConfig = TalonFXConfiguration()
      newConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
      pivotMotor.configurator.apply(newConfig)
    }
  }

  companion object {
    fun createIntakePivot() {
      val pivotMotor = createKraken(
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
