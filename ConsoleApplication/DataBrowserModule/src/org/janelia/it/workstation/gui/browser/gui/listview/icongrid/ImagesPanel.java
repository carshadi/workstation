package org.janelia.it.workstation.gui.browser.gui.listview.icongrid;

import java.awt.*;
import java.awt.event.*;
import java.util.*;
import java.util.List;
import java.util.Timer;
import java.util.concurrent.atomic.AtomicBoolean;

import javax.swing.*;
import org.janelia.it.jacs.model.domain.ontology.Annotation;

import org.janelia.it.workstation.gui.framework.outline.Annotations;
import org.janelia.it.workstation.gui.framework.session_mgr.SessionMgr;
import org.janelia.it.workstation.gui.util.MouseForwarder;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Self-adjusting grid of images which may be resized together.
 *
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public class ImagesPanel<T,S> extends JScrollPane {

    private static final Logger log = LoggerFactory.getLogger(ImagesPanel.class);

    // Constants
    public static final int MIN_IMAGE_WIDTH = 100;
    public static final int DEFAULT_THUMBNAIL_SIZE = 300;
    public static final int MAX_IMAGE_WIDTH = 1000;
    public static final int MIN_TABLE_HEIGHT = 50;
    public static final int DEFAULT_TABLE_HEIGHT = 200;
    public static final int MAX_TABLE_HEIGHT = 500;
    
    // Listeners
    private KeyListener buttonKeyListener;
    private MouseListener buttonMouseListener;

    // UI Components
    private final HashMap<S, AnnotatedImageButton<T>> buttons = new LinkedHashMap<>();
    private final IconPanel<T,S> iconPanel;
    private final ScrollableGridPanel buttonsPanel;

    // State
    private Map<Long, List<Annotation>> filteredAnnotationMap = new HashMap<>();
    private final AtomicBoolean loadUnloadImagesInterrupt = new AtomicBoolean(false);
    private Double lowestAspectRatio;
    private Integer maxImageWidth = DEFAULT_THUMBNAIL_SIZE;
    private Integer currTableHeight = DEFAULT_TABLE_HEIGHT;
    private Rectangle currViewRect;
    private Timer timer;

    // Listen for scroll events
    private final AdjustmentListener scrollListener = new AdjustmentListener() {
        @Override
        public void adjustmentValueChanged(final AdjustmentEvent e) {
            SwingUtilities.invokeLater(new Runnable() {
                @Override
                public void run() {
                    final JViewport viewPort = getViewport();
                    Rectangle viewRect = viewPort.getViewRect();
                    if (viewRect.equals(currViewRect)) {
                        return;
                    }
                    currViewRect = viewRect;
                    if (!e.getValueIsAdjusting()) {
                        loadUnloadImages();
                    }

                    if (timer != null) {
                        timer.cancel();
                    }

                    timer = new Timer();
                    timer.schedule(new TimerTask() {
                        @Override
                        public void run() {
                            loadUnloadImages();
                        }
                    }, 500);
                }
            });
        }
    };

    public ImagesPanel(IconPanel<T,S> iconPanel) {
        this.iconPanel = iconPanel;
        buttonsPanel = new ScrollableGridPanel();
        setViewportView(buttonsPanel);
        setBorder(BorderFactory.createEmptyBorder());
        if (!SessionMgr.getSessionMgr().isDarkLook()) {
            getViewport().setBackground(Color.white);
        }
    }

    public IconPanel<T,S> getIconPanel() {
        return iconPanel;
    }
    
    /**
     * Returns the button with the given unique id.
     */
    public AnnotatedImageButton getButtonById(S uniqueId) {
        return buttons.get(uniqueId);
    }

    public void setButtonKeyListener(KeyListener buttonKeyListener) {
        this.buttonKeyListener = buttonKeyListener;
    }

    public void setButtonMouseListener(MouseListener buttonMouseListener) {
        this.buttonMouseListener = buttonMouseListener;
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

    public void cancelAllLoads() {
        for (AnnotatedImageButton button : buttons.values()) {
//            if (button instanceof DynamicImageButton) {
//                ((DynamicImageButton) button).cancelLoad();
//            }
        }
    }

    
    /**
     * Create the image buttons, but leave the images unloaded for now.
     */
    public void setImageObjects(List<T> imageObjects) {

        buttons.clear();
        for (Component component : buttonsPanel.getComponents()) {
            if (component instanceof AnnotatedImageButton) {
                AnnotatedImageButton button = (AnnotatedImageButton) component;
                buttonsPanel.remove(button);
            }
        }

        this.lowestAspectRatio = null;

        for (final T imageObject : imageObjects) {
            S imageId = iconPanel.getImageUniqueId(imageObject);
            
            if (buttons.containsKey(imageId)) {
                continue;
            }            

            String filepath = iconPanel.getImageFilepath(imageObject, iconPanel.getCurrImageRole());
            AnnotatedImageButton<T> button = AnnotatedImageButton.create(imageObject, filepath, iconPanel);

            button.setTitleVisible(iconPanel.areTitlesVisible());
            button.setTagsVisible(iconPanel.areTagsVisible());

            if (buttonKeyListener != null) {
                button.addKeyListener(buttonKeyListener);
            }
            if (buttonMouseListener != null) {
                button.addMouseListener(buttonMouseListener);
            }

            button.addMouseListener(new MouseForwarder(this, "AnnotatedImageButton->ImagesPanel"));

            // Disable tab traversal, we will do it ourselves
            button.setFocusTraversalKeysEnabled(false);

            buttons.put(imageId, button);
            buttonsPanel.add(button);
        }
    }

    public void removeImageObject(T imageObject) {
        S imageId = iconPanel.getImageUniqueId(imageObject);
        AnnotatedImageButton button = buttons.get(imageId);
        if (button == null) {
            return; // Button was already removed
        }
        buttonsPanel.remove(button);
        buttons.remove(imageId);
    }

    public void setAnnotations(Annotations annotations) {
//        filteredAnnotationMap = annotations.getFilteredAnnotationMap();
    }

    /**
     * Show the given annotations on the appropriate images.
     */
    public void showAllAnnotations() {
//        for (AnnotatedImageButton button : buttons.values()) {
//            showAnnotationsForEntity(button.getRootedEntity().getEntity().getId());
//        }
    }

    /**
     * Show the given annotations on the appropriate images.
     */
    public void showAnnotationsForEntity(final Long entityId) {
//        SwingUtilities.invokeLater(new Runnable() {
//            @Override
//            public void run() {
//                for (AnnotatedImageButton button : getButtonsByEntityId(entityId)) {
//                    List<OntologyAnnotation> entityAnnotations = filteredAnnotationMap.get(entityId);
//                    button.showAnnotations(entityAnnotations);
//                }
//            }
//        });
    }

    public List<AnnotatedImageButton<T>> getButtonsByUniqueId(S uniqueId) {
        List<AnnotatedImageButton<T>> imageButtons = new ArrayList<>();
        for (AnnotatedImageButton<T> button : buttons.values()) {
            S imageId = iconPanel.getImageUniqueId(button.getImageObject());
            if (imageId.equals(uniqueId)) {
                imageButtons.add(button);
            }
        }
        return imageButtons;
    }

    /**
     * Scale all the images to the given max size.
     */
    public synchronized void setMaxImageWidth(int maxImageWidth) {
        if (maxImageWidth < MIN_IMAGE_WIDTH || maxImageWidth > MAX_IMAGE_WIDTH) {
            return;
        }
        log.trace("setMaxImageWidth: {}", maxImageWidth);
        this.maxImageWidth = maxImageWidth;

        double aspectRatio = lowestAspectRatio == null ? 1.0 : lowestAspectRatio;

        int maxImageHeight = (int) Math.round(maxImageWidth / aspectRatio);

        for (AnnotatedImageButton button : buttons.values()) {
            try {
                button.setImageSize(maxImageWidth, maxImageHeight);
            }
            catch (Exception e) {
                SessionMgr.getSessionMgr().handleException(e);
            }
        }

        recalculateGrid();
    }

    public synchronized void resizeTables(int tableHeight) {
        if (tableHeight < MIN_TABLE_HEIGHT || tableHeight > MAX_TABLE_HEIGHT) {
            return;
        }
        log.trace("resizeTables: {}", tableHeight);
        this.currTableHeight = tableHeight;
        for (AnnotatedImageButton button : buttons.values()) {
            try {
                button.resizeTable(tableHeight);
            }
            catch (Exception e) {
                SessionMgr.getSessionMgr().handleException(e);
            }
        }
    }

    public synchronized void showAllButtons() {
        buttonsPanel.removeAll();
        for (AnnotatedImageButton button : buttons.values()) {
            buttonsPanel.add(button);
        }
        revalidate();
        repaint();
    }

    public synchronized void hideButtons(Collection<S> uniqueIds) {
        for (S uniqueId : uniqueIds) {
            AnnotatedImageButton button = getButtonById(uniqueId);
            buttonsPanel.remove(button);
        }
        revalidate();
        repaint();
    }

    public int getMaxImageWidth() {
        return maxImageWidth;
    }

    public int getCurrTableHeight() {
        return currTableHeight;
    }

    public void scrollToBottom() {
        getViewport().scrollRectToVisible(new Rectangle(0, buttonsPanel.getHeight(), 1, 1));
    }

    public void scrollObjectToCenter(T imageObject) {
        if (imageObject == null) {
            return;
        }
        S uniqueId = iconPanel.getImageUniqueId(imageObject);
        AnnotatedImageButton<T> selectedButton = getButtonById(uniqueId);
        scrollButtonToCenter(selectedButton);
    }

    public void scrollButtonToVisible(AnnotatedImageButton button) {
        if (button == null) {
            return;
        }
        getViewport().scrollRectToVisible(button.getBounds());
    }

    public void scrollButtonToCenter(AnnotatedImageButton button) {

        if (button == null) {
            return;
        }
        JViewport viewport = getViewport();

	    // This rectangle is relative to the table where the
        // northwest corner of cell (0,0) is always (0,0).
        Rectangle rect = button.getBounds();

        // The location of the view relative to the table
        Rectangle viewRect = viewport.getViewRect();

	    // Translate the cell location so that it is relative
        // to the view, assuming the northwest corner of the
        // view is (0,0).
        rect.setLocation(rect.x - viewRect.x, rect.y - viewRect.y);

        // Calculate location of rect if it were at the center of view
        int centerX = (viewRect.width - rect.width) / 2;
        int centerY = (viewRect.height - rect.height) / 2;

	    // Fake the location of the cell so that scrollRectToVisible
        // will move the cell to the center
        if (rect.x < centerX) {
            centerX = -centerX;
        }
        if (rect.y < centerY) {
            centerY = -centerY;
        }
        rect.translate(centerX, centerY);

        // Scroll the area into view.
        viewport.scrollRectToVisible(rect);
    }

    public void scrollButtonToTop(AnnotatedImageButton button) {

        if (button == null) {
            return;
        }
        JViewport viewport = getViewport();

	    // This rectangle is relative to the table where the
        // northwest corner of cell (0,0) is always (0,0).
        Rectangle rect = button.getBounds();

        // The location of the view relative to the table
        Rectangle viewRect = viewport.getViewRect();

	    // Translate the cell location so that it is relative
        // to the view, assuming the northwest corner of the
        // view is (0,0). Also make the rect as large as the view, 
        // so that the relevant portion goes to the top.
        rect.setBounds(rect.x - viewRect.x, rect.y - viewRect.y, viewRect.width, viewRect.height);

        // Scroll the area into view.
        viewport.scrollRectToVisible(rect);
    }

    public void scrollSelectedEntitiesToCenter() {
        List<AnnotatedImageButton<T>> selected = getSelectedButtons();
        if (selected.isEmpty()) {
            return;
        }
        int i = selected.size() / 2;
        AnnotatedImageButton centerOfMass = selected.get(i);
        scrollButtonToCenter(centerOfMass);
    }

    public void scrollSelectedEntitiesToTop() {
        List<AnnotatedImageButton<T>> selected = getSelectedButtons();
        if (selected.isEmpty()) {
            return;
        }
        scrollButtonToTop(selected.get(0));
    }

    public synchronized void registerAspectRatio(Double aspectRatio) {
        if (lowestAspectRatio == null || aspectRatio < lowestAspectRatio) {
            this.lowestAspectRatio = aspectRatio;
            // Is this needed? Doesn't seem to be...
            //setMaxImageWidth(maxImageWidth);
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

    public void setTagTable(boolean tagTable) {
//        for (final AnnotatedImageButton button : buttons.values()) {
//            if (tagTable) {
//                if (button.getAnnotationView() instanceof AnnotationTablePanel) {
//                    return;
//                }
//                button.setAnnotationView(new AnnotationTablePanel());
//            }
//            else {
//                if (button.getAnnotationView() instanceof AnnotationTagCloudPanel) {
//                    return;
//                }
//                button.setAnnotationView(new AnnotationTagCloudPanel() {
//                    @Override
//                    protected void moreDoubleClicked(MouseEvent e) {
//                        new EntityDetailsDialog().showForRootedEntity(button.getRootedEntity());
//                    }
//                });
//            }
//        }
    }
    
    private boolean setSelection(final AnnotatedImageButton button, boolean selection) {
        if (button.isSelected() != selection) {
            button.setSelected(selection);
            return true;
        }
        return false;
    }

    public void setSelection(T selectedObject, boolean selection, boolean clearAll) {
        if (clearAll) {
            for (AnnotatedImageButton<T> button : buttons.values()) {
                T imageObject = button.getImageObject();
                if (imageObject.equals(selectedObject)) {
                    setSelection(button, true);
                }
                else {
                    setSelection(button, false);
                }
            }
        }
        else {
            S selectedId = iconPanel.getImageUniqueId(selectedObject);
            AnnotatedImageButton button = buttons.get(selectedId);
            if (button != null) {
                setSelection(button, selection);
            }
        }
    }
    
    public void setSelectedObjects(Set<T> selectedObjects) {
        for (AnnotatedImageButton<T> button : buttons.values()) {
            T imageObject = button.getImageObject();
            if (selectedObjects.contains(imageObject)) {
                setSelection(button, true);
            }
            else {
                setSelection(button, false);
            }
        }
    }
    
    public void setSelectionByUniqueId(S selectedId, boolean selection, boolean clearAll) {
        if (clearAll) {
            for (AnnotatedImageButton<T> button : buttons.values()) {
                T imageObject = button.getImageObject();
                Object imageId = iconPanel.getImageUniqueId(imageObject);
                if (imageId.equals(selectedId)) {
                    setSelection(button, true);
                }
                else {
                    setSelection(button, false);
                }
            }
        }
        else {
            AnnotatedImageButton button = buttons.get(selectedId);
            if (button != null) {
                setSelection(button, selection);
            }
        }
    }

    public List<AnnotatedImageButton<T>> getSelectedButtons() {
        List<AnnotatedImageButton<T>> selected = new ArrayList<>();
        for (AnnotatedImageButton<T> button : buttons.values()) {
            if (button.isSelected()) {
                selected.add(button);
            }
        }
        return selected;
    }

    public void repaintButtons() {
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                buttonsPanel.revalidate();
                buttonsPanel.repaint();
            }
        });
    }

    /**
     * Set the number of columns in the grid layout based on the width of the parent component and the width of the
     * buttons.
     *
     * This method must be called in the EDT.
     */
    public synchronized void recalculateGrid() {

        log.trace("Recalculating image grid");

        double maxButtonWidth = 0;
        for (AnnotatedImageButton button : buttons.values()) {
            int w = button.getPreferredSize().width;
            if (w > maxButtonWidth) {
                maxButtonWidth = w;
            }
        }

        // Should not be needed, but just in case, lets make sure we never divide by zero
        if (maxButtonWidth == 0) {
            maxButtonWidth = 400;
        }

        int fullWidth = getSize().width - getVerticalScrollBar().getWidth();

        int numCols = (int) Math.max(Math.floor((double) fullWidth / maxButtonWidth), 1);
        if (buttonsPanel.getColumns() != numCols) {
            buttonsPanel.setColumns(numCols);
            repaintButtons();
        }

        loadUnloadImages();
    }

    private Date lastQueueDate = new Date();

    public void loadUnloadImages() {
        loadUnloadImagesInterrupt.set(true);
        final Date queueDate = new Date();
        lastQueueDate = queueDate;
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                if (queueDate.before(lastQueueDate)) {
                    log.trace("Ignoring duplicate loadUnloadImages request");
                    return;
                }
                log.trace("Running loadUnloadImages");
                loadUnloadImagesInterrupt.set(false);
                final JViewport viewPort = getViewport();
                Rectangle viewRect = viewPort.getViewRect();
                if (buttonsPanel.getColumns() == 1) {
                    viewRect.setSize(viewRect.width, viewRect.height + 100);
                }
                for (AnnotatedImageButton button : buttons.values()) {
                    if (loadUnloadImagesInterrupt.get()) {
                        log.trace("loadUnloadImages interrupted");
                        return;
                    }
                    try {
                        button.setViewable(viewRect.intersects(button.getBounds()));
                    }
                    catch (Exception e) {
                        SessionMgr.getSessionMgr().handleException(e);
                    }
                }
            }
        });
    }

    private class ScrollableGridPanel extends JPanel implements Scrollable {

        public ScrollableGridPanel() {
            setLayout(new GridLayout(0, 2));
            setBorder(BorderFactory.createEmptyBorder(4, 4, 4, 4));
            setOpaque(false);
            for (ComponentListener l : getComponentListeners()) {
                removeComponentListener(l);
            }
        }

        @Override
        public synchronized void addComponentListener(ComponentListener l) {
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

        public int getColumns() {
            return ((GridLayout) getLayout()).getColumns();
        }

        public void setColumns(int columns) {
            ((GridLayout) getLayout()).setColumns(columns);
        }
    }
}
