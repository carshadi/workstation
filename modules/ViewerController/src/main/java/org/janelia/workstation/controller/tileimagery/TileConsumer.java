package org.janelia.workstation.controller.tileimagery;

import org.janelia.workstation.geom.CoordinateAxis;
import org.janelia.workstation.geom.Rotation3d;
import org.janelia.workstation.gui.camera.Camera3d;
import org.janelia.workstation.gui.viewer3d.interfaces.Viewport;

public interface TileConsumer {
	Camera3d getCamera();
	Viewport getViewport();
	CoordinateAxis getSliceAxis();
	Rotation3d getViewerInGround();
	boolean isShowing();
    void repaint();
}
