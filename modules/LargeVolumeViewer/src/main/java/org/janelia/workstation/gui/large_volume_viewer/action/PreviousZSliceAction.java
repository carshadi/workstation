package org.janelia.workstation.gui.large_volume_viewer.action;

import javax.swing.KeyStroke;

import com.jogamp.newt.event.KeyEvent;

import org.janelia.workstation.common.gui.support.Icons;
import org.janelia.workstation.gui.camera.Camera3d;
import org.janelia.workstation.gui.viewer3d.interfaces.VolumeImage3d;

public class PreviousZSliceAction extends SliceScanAction {
	private static final long serialVersionUID = 1L;

	public PreviousZSliceAction(VolumeImage3d image, Camera3d camera) {
		super(image, camera, -1);
		putValue(NAME, "Previous Z Slice");
		putValue(SMALL_ICON, Icons.getIcon("z_stack_up.png"));
		putValue(MNEMONIC_KEY, (int)KeyEvent.VK_PAGE_UP);
		KeyStroke accelerator = KeyStroke.getKeyStroke(
			KeyEvent.VK_PAGE_UP, 0);
		putValue(ACCELERATOR_KEY, accelerator);
		putValue(SHORT_DESCRIPTION,
				"View previous Z slice"
				+"\n (Shortcut: "+accelerator+")"
				);		
	}

}
