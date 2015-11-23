package org.suhps.simulation.desktop;

import com.badlogic.gdx.backends.lwjgl.LwjglApplication;
import com.badlogic.gdx.backends.lwjgl.LwjglApplicationConfiguration;
import org.suhps.simulation.SubmarineSimulation;

public class DesktopLauncher {
	public static void main (String[] arg) {
		LwjglApplicationConfiguration config = new LwjglApplicationConfiguration();
		config.width = 1280;
		config.height = 768;
		config.useHDPI = true;
		config.vSyncEnabled = true;
		new LwjglApplication(new SubmarineSimulation(), config);
	}
}
