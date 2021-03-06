package org.janelia.workstation.gui.large_volume_viewer;

import javax.media.opengl.GL2;
import javax.media.opengl.GLAutoDrawable;

import org.janelia.workstation.controller.tileimagery.Tile2d;
import org.janelia.workstation.controller.tileimagery.TileSet;
import org.janelia.workstation.controller.tileimagery.ViewTileManager;
import org.janelia.workstation.gui.camera.Camera3d;
import org.janelia.workstation.gui.large_volume_viewer.shader.OutlineShader;
import org.janelia.workstation.gui.opengl.GLActor;
import org.janelia.workstation.geom.BoundingBox3d;
import org.janelia.workstation.gui.viewer3d.shader.AbstractShader;

public class TileOutlineActor 
implements GLActor
{
	private ViewTileManager viewTileManager;
	private OutlineShader outlineShader = new OutlineShader();

	public TileOutlineActor(ViewTileManager viewTileManager) {
		this.viewTileManager = viewTileManager;
	}
	
	@Override
	public void display(GLAutoDrawable glDrawable) {
		display(glDrawable, viewTileManager.createLatestTiles());
	}
	
	private void display(GLAutoDrawable glDrawable, TileSet tiles) {
		if (tiles == null)
			return;
		if (tiles.size() < 1)
			return;
		Camera3d camera = viewTileManager.getTileConsumer().getCamera();
        GL2 gl = glDrawable.getGL().getGL2();
		outlineShader.load(gl);
		gl.glLineWidth(2.0f);
		for (Tile2d tile : tiles) {
			tile.displayBoundingBox(gl, camera);
			// System.out.println("paint tile outline");
		}
		outlineShader.unload(gl);
	}

	@Override
	public BoundingBox3d getBoundingBox3d() {
		return viewTileManager.getVolumeImage().getBoundingBox3d();
	}

	@Override
	public void init(GLAutoDrawable glDrawable) {
		try {
	        GL2 gl = glDrawable.getGL().getGL2();
			outlineShader.init(gl);
		} catch (AbstractShader.ShaderCreationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@Override
	public void dispose(GLAutoDrawable glDrawable) {
		// TODO shader?
	}

}
