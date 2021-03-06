package org.janelia.workstation.gui.opengl.stereo3d;

import org.janelia.workstation.gui.opengl.GLActorContext;
import org.janelia.workstation.gui.opengl.GLSceneComposer;

import javax.media.opengl.GLAutoDrawable;

public class RightEyeStereoMode extends BasicStereoMode
{
    @Override
    public void display(GLActorContext actorContext,
            GLSceneComposer composer)
    {
        GLAutoDrawable glDrawable = actorContext.getGLAutoDrawable();
        updateViewport(glDrawable);
        setRightEyeView(actorContext, composer.getCameraScreenGeometry());
        composer.displayScene(actorContext);
    }

}
