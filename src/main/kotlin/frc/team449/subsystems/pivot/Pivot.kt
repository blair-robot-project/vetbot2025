package frc.team449.subsystems.pivot

import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.system.motor.createKraken
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.subsystems.superstructure.SuperstructureGoal
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.cos

@Logged
class Pivot (
  val motor: TalonFX,
) : SubsystemBase() {

  val positionSupplier = Supplier { motor.position.valueAsDouble }

  private val request: MotionMagicVoltage = MotionMagicVoltage(
    SuperstructureGoal.STOW.pivot.`in`(Radians)
  ).withEnableFOC(false)

  fun setPosition(position: Double): Command {
    return this.runOnce {
      motor.setControl(
        request
          .withPosition(position)
          .withUpdateFreqHz(PivotConstants.REQUEST_UPDATE_RATE)
      )
    }
  }

  fun hold(): Command {
    return this.runOnce {
      motor.setControl(
        PositionVoltage(positionSupplier.get())
          .withUpdateFreqHz(PivotConstants.REQUEST_UPDATE_RATE)
      )
    }
  }

  fun atSetpoint(): Boolean {
    return (abs(motor.position.valueAsDouble - request.Position) < PivotConstants.TOLERANCE.`in`(Radians))
  }

  fun stop(): Command {
    return runOnce {
      motor.stopMotor()
    }
  }

  companion object {
    fun createPivot() : Pivot {
      val pivotMotor = createKraken(
        PivotConstants.PIVOT_ID,
        PivotConstants.PIVOT_INVERTED,
        kS = PivotConstants.KS,
        kV = PivotConstants.KV,
        kG = PivotConstants.KG,
        kP = PivotConstants.KP,
        kI = PivotConstants.KI,
        kD = PivotConstants.KD,
        gravityType = GravityTypeValue.Arm_Cosine,
        cruiseVel = PivotConstants.PIVOT_CRUISE_VEL.`in`(RotationsPerSecond),
        maxAccel = PivotConstants.PIVOT_MAX_ACCEL.`in`(RotationsPerSecondPerSecond)
      )
      return Pivot(pivotMotor)
    }
  }
}
