package org.janelia.it.FlyWorkstation.gui.framework.console;

import java.awt.Component;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.Rectangle;
import java.awt.event.AdjustmentEvent;
import java.awt.event.AdjustmentListener;
import java.awt.event.FocusListener;
import java.awt.event.KeyListener;
import java.util.HashMap;
import java.util.List;

import javax.swing.*;

import org.janelia.it.FlyWorkstation.gui.framework.outline.Annotations;
import org.janelia.it.FlyWorkstation.gui.framework.session_mgr.SessionMgr;
import org.janelia.it.jacs.model.entity.Entity;
import org.janelia.it.jacs.model.ontology.OntologyAnnotation;

/**
 * Self-adjusting grid of images which may be resized together.
 *
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public class ImagesPanel extends JScrollPane {

    private static final int MIN_THUMBNAIL_SIZE = 64;
    public static final int MAX_THUMBNAIL_SIZE = 300;

    private final HashMap<String, AnnotatedImageButton> buttons = new HashMap<String, AnnotatedImageButton>();

    private final ImageCache imageCache = new ImageCache();
    
    private KeyListener buttonKeyListener;
    private FocusListener buttonFocusListener;
    
    private JPanel buttonsPanel;
    private ButtonGroup buttonGroup;

    // Listen for scroll events
    private final AdjustmentListener scrollListener = new AdjustmentListener() {
        @Override
        public void adjustmentValueChanged(final AdjustmentEvent e) {
            SwingUtilities.invokeLater(new Runnable() {
    			@Override
    			public void run() {
    	        	loadUnloadImages();
    			}
    		});
        }
    };
    
    public ImagesPanel() {
    	buttonsPanel = new ScrollableGridPanel();
        setViewportView(buttonsPanel);
    }


	/**
     * Returns the button with the given entity.
     *
     * @return
     */
    public AnnotatedImageButton getButtonByEntityId(long entityId) {
        return buttons.get(entityId+"");
    }
    
    /**
     * TODO: remove this after refactoring so that its not needed.
     */
    public HashMap<String, AnnotatedImageButton> getButtons() {
        return buttons;
    }
    
    public void setButtonKeyListener(KeyListener buttonKeyListener) {
		this.buttonKeyListener = buttonKeyListener;
	}

	public void setButtonFocusListener(FocusListener buttonFocusListener) {
		this.buttonFocusListener = buttonFocusListener;
	}

	public void setScrollLoadingEnabled(boolean enabled) {
		if (enabled) {
			// Reset scrollbar and re-add the listener
	        SwingUtilities.invokeLater(new Runnable() {
				@Override
				public void run() {
					getVerticalScrollBar().setValue(0); 
			        getVerticalScrollBar().addAdjustmentListener(scrollListener);
				}
			});
		}
		else {
	    	// Remove the scroll listener so that we don't get a bunch of bogus events as things are added to the imagesPanel
	    	getVerticalScrollBar().removeAdjustmentListener(scrollListener);
		}
	}
	
	/**
     * Create the image buttons, but leave the images unloaded for now.
     */
    public void setEntities(List<Entity> entities) {
    	
    	// Cancel all loading images
        for (AnnotatedImageButton button : buttons.values()) {
        	button.cancelLoad();
        }

        buttons.clear();
        buttonGroup = new ButtonGroup();
        for (Component component : buttonsPanel.getComponents()) {
            if (component instanceof AnnotatedImageButton) {
            	AnnotatedImageButton button = (AnnotatedImageButton)component;
            	buttonsPanel.remove(button);
            }
        }

        for (int i = 0; i < entities.size(); i++) {
            final Entity entity = entities.get(i);

         	AnnotatedImageButton button = new AnnotatedImageButton(entity);
            button.setCache(imageCache);
            if (buttonKeyListener!=null) button.addKeyListener(buttonKeyListener);
            if (buttonFocusListener!=null) button.addFocusListener(buttonFocusListener);
            
            buttons.put(entity.getId().toString(), button);
            buttonGroup.add(button);
            buttonsPanel.add(button);
        }
        
    }

    /**
     * Show the given annotations on the appropriate images.
     */
    public void loadAnnotations(Annotations annotations) {
        for (AnnotatedImageButton button : buttons.values()) {
        	loadAnnotations(annotations, button.getEntity());
        }
    }

    /**
     * Show the given annotations on the appropriate images.
     */
    public void loadAnnotations(Annotations annotations, Entity entity) {
    	AnnotatedImageButton button = getButtonByEntityId(entity.getId());
    	List<OntologyAnnotation> entityAnnotations = annotations.getFilteredAnnotationMap().get(entity.getId());
        button.getTagPanel().setTags(entityAnnotations);
    }
    
    /**
     * Scale all the images to the desired percent of their true size.
     *
     * @param imageSizePercent
     */
    public void rescaleImages(double imageSizePercent) {
        if (imageSizePercent < 0 || imageSizePercent > 1) {
            return;
        }
        double range = (double) (MAX_THUMBNAIL_SIZE - MIN_THUMBNAIL_SIZE);
        rescaleImages(MIN_THUMBNAIL_SIZE + (int) (range * imageSizePercent));
    }

    /**
     * Scale all the images to the given max size.
     *
     * @param imageSize
     */
    public synchronized void rescaleImages(int imageSize) {
        if (imageSize < MIN_THUMBNAIL_SIZE || imageSize > MAX_THUMBNAIL_SIZE) {
            return;
        }
        for (AnnotatedImageButton button : buttons.values()) {
        	try {
                button.rescaleImage(imageSize);
	    	}
	    	catch (Exception e) {
	    		SessionMgr.getSessionMgr().handleException(e);
	    	}
        }
		buttonsPanel.revalidate();
		buttonsPanel.repaint();
    }

    public void scrollEntityToVisible(Entity entity) {
    	AnnotatedImageButton selectedButton = getButtonByEntityId(entity.getId());
    	if (selectedButton!=null) {
    		buttonsPanel.scrollRectToVisible(selectedButton.getBounds());
    	}
    }
    
    public void setTitleVisbility(boolean visible) {
        for (AnnotatedImageButton button : buttons.values()) {
            button.setTitleVisible(visible);
        }
    }

    public void setTagVisbility(boolean visible) {
        for (AnnotatedImageButton button : buttons.values()) {
            button.setTagsVisible(visible);
        }
    }

    public void setInvertedColors(boolean inverted) {
        for (AnnotatedImageButton button : buttons.values()) {
            button.setInvertedColors(inverted);
        }
    }
    
    public boolean setSelectedImage(Entity entity) {
        AnnotatedImageButton button = buttons.get(entity.getId().toString());
        if (button != null) {
	        button.setSelected(true);
	        if (!button.isFocusOwner()) {
	        	button.requestFocusInWindow();
	        }
	        if (!button.isShowing()) {
	        	scrollRectToVisible(button.getBounds());
	        }
	        return true;
        }
        return false;
    }

    /**
     * Set the number of columns in the grid layout based on the width of the parent component and the width of the
     * buttons.
     */
    public synchronized void recalculateGrid() {
    	
    	if (!SwingUtilities.isEventDispatchThread()) throw new RuntimeException("recalculateGrid called outside of EDT");
    	
    	double maxButtonWidth = 0;
        for (AnnotatedImageButton button : buttons.values()) {
            int w = button.getPreferredSize().width;
            if (w > maxButtonWidth) maxButtonWidth = w;
        }

        // Should not be needed, but just in case, lets make sure we never divide by zero
        if (maxButtonWidth == 0) maxButtonWidth = 400;
        
        int fullWidth = getSize().width - getVerticalScrollBar().getWidth();
        int numCols = (int) Math.floor((double)fullWidth / maxButtonWidth);
        if (numCols > 0) {
            ((GridLayout)buttonsPanel.getLayout()).setColumns(numCols);
        }

        buttonsPanel.revalidate();
        buttonsPanel.repaint();
    }

    public synchronized void loadUnloadImages() {

    	if (!SwingUtilities.isEventDispatchThread()) throw new RuntimeException("recalculateGrid called outside of EDT");
        final JViewport viewPort = getViewport();
    	Rectangle viewRect = viewPort.getViewRect();
		
        for(AnnotatedImageButton button : buttons.values()) {
        	try {
        		button.setViewable(viewRect.intersects(button.getBounds()));
        	}
        	catch (Exception e) {
        		SessionMgr.getSessionMgr().handleException(e);
        	}
        }
    }
    
    private class ScrollableGridPanel extends JPanel implements Scrollable  {

		public ScrollableGridPanel() {
			setLayout(new GridLayout(0, 2));
			setOpaque(false);
		}
		
        @Override
        public Dimension getPreferredScrollableViewportSize() {
            return getPreferredSize();
        }

        @Override
        public int getScrollableUnitIncrement(Rectangle visibleRect, int orientation, int direction) {
            return 30;
        }

        @Override
        public int getScrollableBlockIncrement(Rectangle visibleRect, int orientation, int direction) {
            return 300;
        }

        @Override
        public boolean getScrollableTracksViewportWidth() {
            return false;
        }

        @Override
        public boolean getScrollableTracksViewportHeight() {
            return false;
        }
    }
    

}