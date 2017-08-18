package org.janelia.it.workstation.browser.gui.listview.icongrid;

import java.awt.Dimension;
import java.util.concurrent.Callable;

import javax.swing.JComponent;

import org.janelia.it.workstation.browser.events.selection.SelectionModel;

/**
 * An AnnotatedImageButton with a dynamic image, i.e. one that is loaded
 * from via the network, not a locally available icon.
 *
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public class DynamicImageButton<T,S> extends AnnotatedImageButton<T,S> {

    private DynamicImagePanel dynamicImagePanel;

    public DynamicImageButton(T imageObject, ImageModel<T,S> imageModel, SelectionModel<T,S> selectionModel, ImagesPanel<T,S> imagesPanel, String filepath) {
        super(imageObject, imageModel, selectionModel, imagesPanel, filepath);
    }

    public JComponent init(T imageObject, ImageModel<T,S> imageModel, String filepath) {
        this.dynamicImagePanel = new DynamicImagePanel(filepath, imageModel.getDecorators(imageObject), ImagesPanel.MAX_IMAGE_WIDTH);
        return dynamicImagePanel;
    }

    public void cancelLoad() {
        dynamicImagePanel.cancelLoad();
    }

    @Override
    public void setImageSize(int width, int height) {
        super.setImageSize(width, height);
        dynamicImagePanel.rescaleImage(width);
        dynamicImagePanel.setPreferredSize(new Dimension(width, height));
    }

    @Override
    public void setViewable(boolean viewable) {
        dynamicImagePanel.setViewable(viewable, new Callable<Void>() {
            @Override
            public Void call() throws Exception {
                // Register our image height
                if (dynamicImagePanel.getMaxSizeImage() != null && dynamicImagePanel.getImage() != null) {
                    double w = dynamicImagePanel.getWidth();
                    double h = dynamicImagePanel.getHeight();
                    registerAspectRatio(w, h);
                }
                return null;
            }

        });
    }
    
    // TODO: in the future, we may want to display titles directly on the image. 
    // Currently disabled, because it will take a bit more work to finish the implementation.
    
//    @Override
//    public void setTitle(String title, int maxWidth) {
//        dynamicImagePanel.setTitle(title);
//    }
//
//    @Override
//    public void setSubtitle(String subtitle, int maxWidth) {
//        dynamicImagePanel.setSubtitle(subtitle);
//    }
    
}
