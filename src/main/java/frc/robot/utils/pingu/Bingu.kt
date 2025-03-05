package frc.robot.utils.pingu

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.commands.Kommand.cmd
import frc.robot.utils.emu.Button

/**
 * BindPingu is a utility object for binding Xbox controller buttons to commands.
 */
object Bingu : SubsystemBase() {
    // List to store the mappings of controllers, buttons, and command suppliers
    private val buttonMaps: MutableList<Triple<XboxController, Button, () -> Command>> = mutableListOf()

    /**
     * Extension function for XboxController to bind multiple button-command pairs.
     *
     * @param pair Vararg of pairs where each pair consists of a Button and a command supplier function.
     */
    @JvmStatic
    @SafeVarargs
    fun XboxController.bindings(vararg pair: Pair<Button, () -> Command>) {
        pair.forEach { (button, commandSupplier) ->
            buttonMaps.add(Triple(this, button, commandSupplier))
        }
    }

    /**
     * Creates a pair of a Button and a command supplier function.
     *
     * @param button The button to bind.
     * @param commandSupplier A function that supplies the command to be executed when the button is pressed.
     * @return A pair of the button and the command supplier.
     */
    @JvmStatic
    fun bind(
        button: Button,
        commandSupplier: () -> Command,
    ) = button to commandSupplier

    /**
     * Creates a pair of a Button and a command supplier function using a runnable.
     *
     * @param button The button to bind.
     * @param runnable A function to be executed when the button is pressed.
     * @return A pair of the button and the created command supplier.
     */
    @JvmStatic
    fun bind(
        button: Button,
        runnable: Runnable,
    ) = button to cmd { runnable.run() }

    /**
     * Periodically checks the state of each button and schedules the corresponding command if the button is pressed.
     */
    override fun periodic() {
        buttonMaps.forEach { (controller, button, commandSupplier) ->
            if (button.checkPressed(controller)) {
                commandSupplier().schedule()
            }
        }
    }
}
