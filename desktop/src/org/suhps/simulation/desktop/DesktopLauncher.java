package org.suhps.simulation.desktop;

import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import org.suhps.simulation.SubmarineSimulation;

public class DesktopLauncher {
	public static void main (String[] arg) {
		Lwjgl3ApplicationConfiguration config = new Lwjgl3ApplicationConfiguration();
        config.setTitle("Submarine Simulation");
        config.setBackBufferConfig(8, 8, 8, 8, 16, 0, 4);
        config.setWindowedMode(800, 600);
		new Lwjgl3Application(new SubmarineSimulation(), config);
	}
}
