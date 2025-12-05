package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.trajectory.SwerveSample
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.Robot

open class Routines(
  val robot: Robot
) {
  private val autoFactory =
    AutoFactory(
      robot.poseSubsystem::pose,
      robot.poseSubsystem::resetOdometry,
      { sample: SwerveSample -> robot.drive.followTrajectory(robot, sample) },
      false,
      robot.drive,
    )

  // do nothing
  fun doNothing(): AutoRoutine {
    val nothing: AutoRoutine = autoFactory.newRoutine("Nothing")
    return nothing
  }

  fun test(): AutoRoutine {
    val testing: AutoRoutine = autoFactory.newRoutine("test")
    val path = testing.trajectory("test")
    testing.active().onTrue(
      Commands.sequence(
        path.resetOdometry(),
        path.cmd(),
      ),
    )
    path.done().onTrue(robot.drive.driveStop())
    return testing
  }

/** only shoot preload, no moving**/
  private fun shoot(): AutoRoutine {
    val shooting: AutoRoutine = autoFactory.newRoutine("Shoot")
    shooting.active().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        WaitCommand(0.5),
        robot.superstructureManager.shootHigh(),
      ),
    )
    return shooting
  }

  /** shoot preload -> then taxi **/
  private fun shootAndTaxi(alliance: String): AutoRoutine {
    val taxi: AutoRoutine = autoFactory.newRoutine("${if (alliance == "B") "Blue" else "Red"} Shoot and Taxi")
    val taxiPath = taxi.trajectory("taxi($alliance)")
    taxi.active().onTrue(
      Commands.sequence(
        taxiPath.resetOdometry(),
        WaitCommand(0.5),
        // robot.superstructureManager.shootHigh(),
        taxiPath.cmd(),
        robot.drive.driveStop(),
      ),
    )
    return taxi
  }

/** start at far goal, shoot preload -> then taxi**/
  private fun shootAndTaxiFar(alliance: String): AutoRoutine {
    val taxi: AutoRoutine = autoFactory.newRoutine("${if (alliance == "B") "Blue" else "Red"} Far Shoot & Taxi")
    val taxiPath = taxi.trajectory("farTaxi($alliance)")
    taxi.active().onTrue(
      Commands.sequence(
        taxiPath.resetOdometry(),
        WaitCommand(0.5),
        //   robot.superstructureManager.shootHigh(),
        taxiPath.cmd(),
        robot.drive.driveStop(),
      ),
    )
    return taxi
  }

/** shoot preload -> intake 3 from ground -> go back to goal and shoot -> taxi**/
  private fun high3(alliance: String): AutoRoutine {
    val threeHigh: AutoRoutine = autoFactory.newRoutine("${if (alliance == "B") "Blue" else "Red" } 3 High")
    val goToIntake = threeHigh.trajectory("1($alliance)")
    val back = threeHigh.trajectory("2($alliance)")
    val taxi = threeHigh.trajectory("3($alliance)")
    threeHigh.active().onTrue(
      Commands.sequence(
        goToIntake.resetOdometry(),
        //   robot.superstructureManager.shootHigh(),
        goToIntake.cmd(), // .alongWith(
        //   robot.superstructureManager.prepIntake().andThen(
        //      robot.superstructureManager.intake())
        // ),
        back.cmd(), // .alongWith(robot.superstructureManager.stow()),
        // robot.superstructureManager.shootHigh(),
        taxi.cmd(),
        robot.drive.driveStop(),
      ),
    )
    return threeHigh
  }

  fun shootAndTaxiBlue(): AutoRoutine = shootAndTaxi("B")

  fun shootAndTaxiRed(): AutoRoutine = shootAndTaxi("R")

  fun shootAndTaxiFarBlue(): AutoRoutine = shootAndTaxiFar("B")

  fun shootAndTaxiFarRed(): AutoRoutine = shootAndTaxiFar("R")

  fun high3Blue(): AutoRoutine = high3("B")

  fun high3Red(): AutoRoutine = high3("R")

  // autoChooser that will be displayed on dashboard
  fun addOptions(autoChooser: AutoChooser) {
    autoChooser.addRoutine("Do Nothing", this::doNothing)
    autoChooser.addRoutine("Only Shoot", this::shoot)
    autoChooser.addRoutine("Blue high 3", this::high3Blue)
    autoChooser.addRoutine("Red high 3", this::high3Red)
    autoChooser.addRoutine("Blue Taxi", this::shootAndTaxiBlue)
    autoChooser.addRoutine("Red Taxi", this::shootAndTaxiRed)
    autoChooser.addRoutine("Blue Far Taxi", this::shootAndTaxiFarBlue)
    autoChooser.addRoutine("Red Far Taxi", this::shootAndTaxiFarRed)
    autoChooser.addRoutine("test", this::test)
  }
}
