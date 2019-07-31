package org.janelia.workstation.browser.gui.components;

import com.google.common.eventbus.Subscribe;
import org.janelia.model.domain.DomainObject;
import org.janelia.model.domain.DomainUtils;
import org.janelia.model.domain.Reference;
import org.janelia.model.domain.gui.cdmip.ColorDepthMatch;
import org.janelia.model.domain.sample.LSMImage;
import org.janelia.model.domain.sample.NeuronFragment;
import org.janelia.model.domain.sample.Sample;
import org.janelia.workstation.browser.gui.editor.SampleEditorPanel;
import org.janelia.workstation.common.gui.editor.DomainObjectEditor;
import org.janelia.workstation.common.gui.util.UIUtils;
import org.janelia.workstation.core.actions.ViewerContext;
import org.janelia.workstation.core.api.AccessManager;
import org.janelia.workstation.core.api.DomainMgr;
import org.janelia.workstation.core.events.Events;
import org.janelia.workstation.core.events.lifecycle.SessionStartEvent;
import org.janelia.workstation.core.events.selection.DomainObjectSelectionEvent;
import org.janelia.workstation.core.events.selection.ViewerContextChangeEvent;
import org.janelia.workstation.core.nodes.ChildObjectsNode;
import org.janelia.workstation.core.workers.SimpleWorker;
import org.janelia.workstation.integration.util.FrameworkAccess;
import org.netbeans.api.settings.ConvertAsProperties;
import org.openide.awt.ActionID;
import org.openide.util.NbBundle.Messages;
import org.openide.util.lookup.AbstractLookup;
import org.openide.util.lookup.InstanceContent;
import org.openide.windows.TopComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.swing.JComponent;
import javax.swing.SwingUtilities;
import java.awt.BorderLayout;
import java.awt.Component;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * Top component which displays domain object viewers. 
 * 
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
@ConvertAsProperties(
        dtd = "-//org.janelia.workstation.browser.components//DomainViewer//EN",
        autostore = false
)
@TopComponent.Description(
        preferredID = DomainViewerTopComponent.TC_NAME,
        //iconBase="SET/PATH/TO/ICON/HERE", 
        persistenceType = TopComponent.PERSISTENCE_ALWAYS
)
@TopComponent.Registration(mode = "editor2", openAtStartup = false)
@ActionID(category = "Window", id = "org.janelia.workstation.browser.components.DomainViewerTopComponent")
//@ActionReference(path = "Menu/Window" /*, position = 333 */)
@TopComponent.OpenActionRegistration(
        displayName = "#CTL_DomainViewerAction",
        preferredID = DomainViewerTopComponent.TC_NAME
)
@Messages({
    "CTL_DomainViewerAction=Domain Object Viewer",
    "CTL_DomainViewerTopComponent=Domain Object Viewer",
    "HINT_DomainViewerTopComponent=Domain Object Viewer"
})
public final class DomainViewerTopComponent extends TopComponent {

    private static final Logger log = LoggerFactory.getLogger(DomainViewerTopComponent.class);
    
    public static final String TC_NAME = "DomainViewerTopComponent";
    public static final String TC_VERSION = "1.0";
        
    /* Instance variables */
    
    private final InstanceContent content = new InstanceContent();
    private DomainObjectEditor<DomainObject> editor;
    private Reference refToOpen;
    
    public DomainViewerTopComponent() {
        initComponents();
        setName(Bundle.CTL_DomainViewerTopComponent());
        setToolTipText(Bundle.HINT_DomainViewerTopComponent());
        associateLookup(new AbstractLookup(content));
        // Init the viewer manager
        SampleResultViewerManager.getInstance();
    }

    /**
     * This method is called from within the constructor to
     * initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is
     * always regenerated by the Form Editor.
     */
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        setLayout(new java.awt.BorderLayout());
    }// </editor-fold>//GEN-END:initComponents

    // Variables declaration - do not modify//GEN-BEGIN:variables
    // End of variables declaration//GEN-END:variables
    
    @Override
    public void componentOpened() {
        log.debug("componentOpened - {}", this.getName());
        Events.getInstance().registerOnEventBus(this);
    }

    @Override
    public void componentClosed() {
        log.debug("componentClosed - {}", this.getName());
        clearEditor();
        Events.getInstance().unregisterOnEventBus(this);
    }

    @Override
    protected void componentShowing() {
        log.debug("componentShowing - {}", this.getName());
        DomainViewerManager.getInstance().activate(this);
    }

    @Override
    protected void componentHidden() {
        log.debug("componentHidden - {}", this.getName());
    }

    @Override
    protected void componentActivated() {
        log.debug("componentActivated - {}", this.getName());
        DomainViewerManager.getInstance().activate(this);
        if (editor!=null) {
            editor.activate();
            ViewerUtils.updateContextIfChanged(this, content, editor.getViewerContext());
            if (editor.getViewerContext()!=null) {
                ViewerUtils.updateNodeIfChanged(this, content, editor.getViewerContext().getSelectionModel().getObjects());
            }
        }
    }
    
    @Override
    protected void componentDeactivated() {
        log.debug("componentDeactivated - {}", this.getName());
        if (editor!=null) {
            editor.deactivate();
        }
    }

    @Subscribe
    public void selectedChanged(DomainObjectSelectionEvent e) {
        // Make sure this selection comes from one of our children, otherwise there's no point in updating anything
        TopComponent topComponent = UIUtils.getAncestorWithType(
                (Component)e.getSourceComponent(), TopComponent.class);
        if (topComponent==this && editor!=null) {
            log.info("Our selection changed, updating cookie because of {}", e);
            if (editor.getViewerContext()!=null) {
                ViewerUtils.updateContextIfChanged(this, content, editor.getViewerContext());
                if (editor.getViewerContext()!=null) {
                    ViewerUtils.updateNodeIfChanged(this, content, editor.getViewerContext().getSelectionModel().getObjects());
                }
            }
        }
    }

    // TODO: complete this refactoring so that context menus can be unified

    //    @Subscribe
//    public void resultSelected(PipelineResultSelectionEvent e) {
//        TopComponent topComponent = UIUtils.getAncestorWithType(
//                (Component)e.getSourceComponent(), TopComponent.class);
//        if (topComponent==this && editor!=null) {
//            PipelineResult result = e.getPipelineResult();
//            updateNodeIfChanged(result);
//        }
//    }
//
//    @Subscribe
//    public void errorSelected(PipelineErrorSelectionEvent e) {
//        TopComponent topComponent = UIUtils.getAncestorWithType(
//                (Component)e.getSourceComponent(), TopComponent.class);
//        if (topComponent==this && editor!=null) {
//            PipelineError result = e.getPipelineError();
//            updateNodeIfChanged(result);
//        }
//    }

    @Subscribe
    public void contextChanged(ViewerContextChangeEvent e) {
        // Make sure this selection comes from one of our children, otherwise there's no point in updating anything
        TopComponent topComponent = UIUtils.getAncestorWithType(
                (Component)e.getSourceComponent(), TopComponent.class);
        if (topComponent==this && editor!=null) {
            log.info("Viewer context changed, updating cookie because of {}", e);
            ViewerUtils.updateContextIfChanged(this, content, editor.getViewerContext());
        }
    }

//    private void updateContextIfChanged(ViewerContext viewerContext) {
//        Collection<? extends ViewerContext> viewerContexts = getLookup().lookupAll(ViewerContext.class);
//        if (viewerContexts.isEmpty() || viewerContexts.iterator().next().equals(viewerContext)) {
//            // Clear all existing nodes
//            getLookup().lookupAll(ViewerContext.class).forEach(content::remove);
//            // Add new node
//            if (viewerContext!=null) {
//                content.add(viewerContext);
//            }
//        }
//    }
//
//    private void updateNodeIfChanged(Collection objects) {
//
//        List<Object> currentObjects = new ArrayList<>();
//        for (ChildObjectsNode childObjectsNode : getLookup().lookupAll(ChildObjectsNode.class)) {
//            currentObjects.addAll(childObjectsNode.getObjects());
//        }
//
//        List<Object> newObjects = new ArrayList<>(objects);
//        if (!currentObjects.equals(newObjects)) {
//            log.info("Updating ChildObjectsNode (current={}, new={})", currentObjects.size(), newObjects.size());
//            // Clear all existing nodes
//            getLookup().lookupAll(ChildObjectsNode.class).forEach(content::remove);
//            // Add new node
//            content.add(new ChildObjectsNode(newObjects));
//        }
//    }

    void writeProperties(java.util.Properties p) {
        if (p==null) return;
        p.setProperty("version", TC_VERSION);
        DomainObject current = getCurrent();
        if (current!=null) {
            String objectRef = Reference.createFor(current).toString();
            log.info("Writing state: {}",objectRef);
            p.setProperty("objectRef", objectRef);
        }
        else {
            p.remove("objectRef");
        }
    }

    void readProperties(java.util.Properties p) {
        if (p==null) return;
        String version = p.getProperty("version");
        final String objectStrRef = p.getProperty("objectRef");
        log.info("Reading state: {}",objectStrRef);
        if (TC_VERSION.equals(version) && objectStrRef!=null) {
            // Must write to instance variables from EDT only
            SwingUtilities.invokeLater(new Runnable() {
                @Override
                public void run() {
                    refToOpen = Reference.createFor(objectStrRef);
                    if (AccessManager.loggedIn()) {
                        loadPreviousSession();
                    }
                    else {
                        // Not logged in yet, wait for a SessionStartEvent
                    }
                }
            });
        }
    }
    
    @Subscribe
    public void sessionStarted(SessionStartEvent event) {
        loadPreviousSession();
    }
    
    private void loadPreviousSession() {
        
        if (refToOpen==null) return;
        log.info("Loading previous session: "+refToOpen);
        final Reference objectRef = refToOpen;
        this.refToOpen = null;
        
        SimpleWorker worker = new SimpleWorker() {
            DomainObject object;

            @Override
            protected void doStuff() throws Exception {
                object = DomainMgr.getDomainMgr().getModel().getDomainObject(objectRef);
            }

            @Override
            protected void hadSuccess() {
                if (object!=null) {
                    loadDomainObject(object, false);
                }
            }

            @Override
            protected void hadError(Throwable error) {
                FrameworkAccess.handleException(error);
            }
        };
        worker.execute();
    }

    public DomainObject getCurrent() {
        return getLookup().lookup(DomainObject.class);
    }

    private boolean setCurrent(DomainObject domainObject) {
        DomainObject curr = getCurrent();
        if (domainObject.equals(curr)) {
            return false;
        }
        if (curr!=null) {
            content.remove(curr);
        }
        content.add(domainObject);
        return true;
    }

    public void clearEditor() {
        if (editor!=null) {
            remove((JComponent)editor);
            Events.getInstance().unregisterOnEventBus(editor.getEventBusListener());
        }
        this.editor = null;
    }
    
    @SuppressWarnings("unchecked")
    public void setEditorClass(Class<? extends DomainObjectEditor<?>> editorClass) {
        try {
            clearEditor();
            editor = (DomainObjectEditor<DomainObject>) editorClass.newInstance();
            add((JComponent)editor, BorderLayout.CENTER);
            Events.getInstance().registerOnEventBus(editor.getEventBusListener());
            revalidate();
            repaint();
        }
        catch (InstantiationException | IllegalAccessException e) {
            FrameworkAccess.handleException(e);
        }
        setName(editor.getName());
    }
    
    public DomainObjectEditor<? extends DomainObject> getEditor() {
        return editor;
    }

    public boolean isCurrent(DomainObject domainObject) {
        try {
            domainObject = DomainViewerManager.getObjectToLoad(domainObject);
            if (domainObject==null) return getCurrent()==null;
            return DomainUtils.equals(getCurrent(), domainObject);
        }
        catch (Exception e) {
            FrameworkAccess.handleException(e);
            return false;
        }
    }

    public void loadDomainObject(DomainObject domainObject, boolean isUserDriven) {
        try {
            domainObject = DomainViewerManager.getObjectToLoad(domainObject);
            log.info("loadDomainObject({}, isUserDriven={})", domainObject, isUserDriven);
            if (domainObject==null) return;
            
            final Class<? extends DomainObjectEditor<?>> editorClass = getEditorClass(domainObject);
            if (editorClass == null) {
                log.debug("No viewer defined for domain object of type {}", domainObject.getClass().getName());
                return;
            }

            // Do we already have the given node loaded?
            if (!setCurrent(domainObject)) {
                log.warn("Already current: {}", domainObject);
                return;
            }

            if (editor == null || !editor.getClass().equals(editorClass)) {
                setEditorClass(editorClass);
            }
            editor.loadDomainObject(domainObject, isUserDriven, null);
            setName(editor.getName());
        }
        catch (Exception e) {
            FrameworkAccess.handleException(e);
        }
    }

    private static Class<? extends DomainObjectEditor<?>> getEditorClass(Object object) {
        if (object instanceof Sample) {
            return SampleEditorPanel.class;
        }
        else if (object instanceof NeuronFragment) {
            return SampleEditorPanel.class;
        }
        else if (object instanceof LSMImage) {
            return SampleEditorPanel.class;
        }
        else if (object instanceof ColorDepthMatch) {
            return SampleEditorPanel.class;
        }
        return null;
    }
    
    public static boolean isSupported(Object object) {
        return object!=null && getEditorClass(object)!=null;
    }
}
