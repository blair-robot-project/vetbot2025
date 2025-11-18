package frc.team449.subsystems.pivot

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.createKraken

@Logged
open class Pivot(
  val pivotMotor: TalonFX,
) : SubsystemBase() {

  val pivotMechanism2d = Mechanism2d(3.0, 3.0, Color8Bit(0, 0,0))
  val pivotRoot = pivotMechanism2d.getRoot("pivotRoot", 0.25, 0.25)
  val pivotLigament = pivotRoot.append(
    MechanismLigament2d(
      "pivotLigament",
      2.0,
      0.0,
      3.0,
      Color8Bit(0, 255, 0)
    )
  )

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

  override fun periodic() {
    pivotLigament.angle = pivotMotor.position.value.`in`(Degrees)
    SmartDashboard.putData("Pivot", pivotMechanism2d)
  }

  companion object {
    fun createIntakePivot(): Pivot {
      val pivotMotor = createKraken(
        PivotConstants.PIVOT_ID,
        PivotConstants.PIVOT_INVERTED,
        sensorToMech = PivotConstants.PIVOT_SENSOR_TO_MECH,
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
