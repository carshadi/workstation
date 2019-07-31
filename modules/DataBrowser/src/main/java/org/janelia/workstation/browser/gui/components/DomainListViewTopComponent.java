package org.janelia.workstation.browser.gui.components;

import java.awt.BorderLayout;
import java.awt.Component;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.Callable;

import javax.swing.JComponent;
import javax.swing.SwingUtilities;

import com.google.common.eventbus.Subscribe;
import org.janelia.model.domain.DomainObject;
import org.janelia.workstation.browser.api.state.DataBrowserMgr;
import org.janelia.workstation.browser.gui.find.FindContext;
import org.janelia.workstation.browser.gui.find.FindContextActivator;
import org.janelia.workstation.browser.gui.find.FindContextManager;
import org.janelia.workstation.common.gui.editor.DomainObjectEditorState;
import org.janelia.workstation.common.gui.editor.ParentNodeSelectionEditor;
import org.janelia.workstation.common.gui.support.MouseForwarder;
import org.janelia.workstation.common.gui.util.UIUtils;
import org.janelia.workstation.core.actions.ViewerContext;
import org.janelia.workstation.core.api.AccessManager;
import org.janelia.workstation.core.api.DomainMgr;
import org.janelia.workstation.core.events.Events;
import org.janelia.workstation.core.events.lifecycle.SessionStartEvent;
import org.janelia.workstation.core.events.selection.DomainObjectSelectionEvent;
import org.janelia.workstation.core.events.selection.ViewerContextChangeEvent;
import org.janelia.workstation.core.nodes.ChildObjectsNode;
import org.janelia.workstation.core.nodes.DomainObjectNode;
import org.janelia.workstation.core.workers.SimpleWorker;
import org.janelia.workstation.integration.spi.domain.DomainObjectHandler;
import org.janelia.workstation.integration.spi.domain.ServiceAcceptorHelper;
import org.janelia.workstation.integration.util.FrameworkAccess;
import org.netbeans.api.settings.ConvertAsProperties;
import org.openide.awt.ActionID;
import org.openide.util.NbBundle.Messages;
import org.openide.util.lookup.AbstractLookup;
import org.openide.util.lookup.InstanceContent;
import org.openide.windows.TopComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Top component which displays lists of domain objects.
 * 
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
@ConvertAsProperties(
        dtd = "-//org.janelia.workstation.browser.components//DomainListView//EN",
        autostore = false
)
@TopComponent.Description(
        preferredID = DomainListViewTopComponent.TC_NAME,
        //iconBase = "images/folder_open_page.png",
        persistenceType = TopComponent.PERSISTENCE_ONLY_OPENED
)
@TopComponent.Registration(mode = "editor", openAtStartup = false)
@ActionID(category = "Window", id = "org.janelia.workstation.browser.components.DomainListViewTopComponent")
@TopComponent.OpenActionRegistration(
        displayName = "#CTL_DomainListViewAction",
        preferredID = DomainListViewTopComponent.TC_NAME
)
@Messages({
    "CTL_DomainListViewAction=Data Browser",
    "CTL_DomainListViewTopComponent=Data Browser"
})
public final class DomainListViewTopComponent extends TopComponent implements FindContextActivator {

    private static final Logger log = LoggerFactory.getLogger(DomainListViewTopComponent.class);

    public static final String TC_NAME = "DomainListViewTopComponent";
    public static final String TC_VERSION_IDONLY = "1.0";
    public static final String TC_VERSION = "1.1";
    
    /* Instance variables */
    
    private final InstanceContent content = new InstanceContent();
    @SuppressWarnings("rawtypes")
    private ParentNodeSelectionEditor editor;
    private FindContext findContext;
    private String intialState;

    public DomainListViewTopComponent() {
        initComponents();
        setName(Bundle.CTL_DomainListViewTopComponent());
        associateLookup(new AbstractLookup(content));
        // Init the viewer manager
        DomainViewerManager.getInstance();
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
        log.info("componentOpened - {}", this.getName());
        // Make this the active list viewer
        DomainListViewManager.getInstance().activate(this);
        // Listen for events
        Events.getInstance().registerOnEventBus(this);
        if (editor!=null) {
            // Activate the child editor
            editor.activate();
        }
    }
    
    @Override
    public void componentClosed() {
        log.debug("componentClosed - {}", this.getName());
        // Stop listening for events
        Events.getInstance().unregisterOnEventBus(this);
        if (editor!=null) {
            // Deactivate the child editor
            editor.deactivate();
        }
        clearEditor();
    }

    @Override
    protected void componentActivated() {
        log.info("componentActivated - {}", this.getName());
        // Make this the active list viewer
        DomainListViewManager.getInstance().activate(this);
        // Take control of the history navigation buttons
        DataBrowserMgr.getDataBrowserMgr().updateNavigationButtons(this);
        // Make our ancestor editor the current find context
        if (findContext!=null) {
            FindContextManager.getInstance().activateContext(findContext);
        }
        // Select the object in the Explorer
        DomainObject domainObject = getCurrent();
        if (DomainExplorerTopComponent.getInstance()!=null && domainObject!=null) {
            DomainExplorerTopComponent.getInstance().selectNodeById(domainObject.getId());
        }
        if (editor!=null) {
            ViewerUtils.updateContextIfChanged(this, content, editor.getViewerContext());
            ViewerUtils.updateNodeIfChanged(this, content, editor.getSelectionModel().getObjects());
        }
    }
    
    @Override
    protected void componentDeactivated() {
        log.debug("componentDeactivated - {}", this.getName());
        if (findContext!=null) {
            FindContextManager.getInstance().deactivateContext(findContext);
        }
    }

    @Subscribe
    public void selectedChanged(DomainObjectSelectionEvent e) {
        // Make sure this selection comes from one of our children, otherwise there's no point in updating anything
        TopComponent topComponent = UIUtils.getAncestorWithType(
                (Component)e.getSourceComponent(), TopComponent.class);
        if (topComponent==this && editor!=null) {
            log.trace("Our selection changed, updating cookie because of {}", e);
            ViewerUtils.updateContextIfChanged(this, content, editor.getViewerContext());
            ViewerUtils.updateNodeIfChanged(this, content, editor.getSelectionModel().getObjects());
        }
    }

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
//            log.trace("Updating ChildObjectsNode (current={}, new={})", currentObjects.size(), newObjects.size());
//            // Clear all existing nodes
//            getLookup().lookupAll(ChildObjectsNode.class).forEach(content::remove);
//            // Add new node
//            content.add(new ChildObjectsNode(newObjects));
//        }
//    }

    void writeProperties(java.util.Properties p) {
        if (p==null || editor==null) return;
        try {
            DomainObjectEditorState<?,?,?> state = editor.saveState();
            String serializedState = ViewerUtils.serialize(state);
            log.info("Writing state: {}",serializedState);
            p.setProperty("version", TC_VERSION);
            p.setProperty("editorState", serializedState);
        }
        catch (Exception e) {
            FrameworkAccess.handleExceptionQuietly("Could not serialize editor state", e);
            p.remove("editorState");
        }
    }

    void readProperties(java.util.Properties p) {
        if (p==null) return;
        String version = p.getProperty("version");
        if (TC_VERSION.equals(version)) {
            // Current version saved the entire editor state
            final String editorState = p.getProperty("editorState");
            log.info("Reading state: {}",editorState);
            if (editorState != null) {
                // Must write to instance variables from EDT only
                SwingUtilities.invokeLater(new Runnable() {
                    @Override
                    public void run() {
                        intialState = editorState;
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
        else if (TC_VERSION_IDONLY.equals(version)) {
            // An older version only saved the object id.
            // This is no longer supported.
        }
        else {
            log.warn("Unrecognized TC version: {}",version);
        }
    }
    
    // Custom methods

    @Subscribe
    public void sessionStarted(SessionStartEvent event) {
        loadPreviousSession();
    }
    
    private void loadPreviousSession() {

        if (intialState == null) return;
        log.info("Loading initial state");
        final String stateToLoad = intialState;
        this.intialState = null;

        SimpleWorker worker = new SimpleWorker() {
            DomainObjectEditorState<?,?,?> state;
            DomainObject domainObject;

            @Override
            protected void doStuff() throws Exception {
                state = ViewerUtils.deserialize(stateToLoad);
                if (state!=null && state.getDomainObject()!=null && state.getDomainObject().getId()!=null) {
                    // Refresh the object, if it's coming from the database
                    domainObject = DomainMgr.getDomainMgr().getModel().getDomainObject(state.getDomainObject());
                    if (domainObject!=null) {
                        state.setDomainObject(domainObject);
                    }
                }
            }

            @Override
            protected void hadSuccess() {
                loadState(state);
            }

            @Override
            protected void hadError(Throwable error) {
                log.warn("Could not load serialized editor state", error);
            }
        };
        worker.execute();
    }
    
    @Override
    public void setFindContext(FindContext findContext) {
        this.findContext = findContext; 
        if (DomainListViewManager.getInstance().isActive(this)) {
            FindContextManager.getInstance().activateContext(findContext);
        }
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
            Events.getInstance().unregisterOnEventBus(editor);
            if (editor.getEventBusListener() != editor) {
                Events.getInstance().unregisterOnEventBus(editor.getEventBusListener());
            }
        }
        this.editor = null;
    }
    
    public void setEditorClass(Class<? extends ParentNodeSelectionEditor<? extends DomainObject,?,?>> editorClass) {
        try {
            clearEditor();
            editor = editorClass.newInstance();
            if (editor.getEventBusListener() != editor) {
                Events.getInstance().registerOnEventBus(editor.getEventBusListener());
            }
            Events.getInstance().registerOnEventBus(editor);
            
            JComponent editorComponent = (JComponent)editor;
            editorComponent.addMouseListener(new MouseForwarder(this, "ParentNodeSelectionEditor->DomainListViewTopComponent"));
            add(editorComponent, BorderLayout.CENTER);
        }
        catch (InstantiationException | IllegalAccessException e) {
            FrameworkAccess.handleException(e);
        }
        setName(editor.getName());
    }
    
    public ParentNodeSelectionEditor<? extends DomainObject,?,?> getEditor() {
        return editor;
    }

    @SuppressWarnings({ "unchecked" })
    public void loadDomainObjectNode(DomainObjectNode<? extends DomainObject> domainObjectNode, boolean isUserDriven) {
        
        log.trace("loadDomainObjectNode({}, isUserDriven={})", domainObjectNode.getDomainObject().getName(), isUserDriven);
        
        if (!prepareForLoad(domainObjectNode.getDomainObject())) return;
        editor.loadDomainObjectNode(domainObjectNode, isUserDriven, afterLoad);
    }

    @SuppressWarnings({ "unchecked" })
    public void loadDomainObject(DomainObject domainObject, boolean isUserDriven) {
        
        log.trace("loadDomainObject({}, isUserDriven={})", domainObject.getName(), isUserDriven);
        
        if (!prepareForLoad(domainObject)) return;
        editor.loadDomainObject(domainObject, isUserDriven, afterLoad);
    }

    /**
     * This callback runs after an editor is loaded. It pushes a placeholder state to the history,
     * and updates the parent component title bar.
     * 
     * The placeholder state will be updated with the latest state when the user navigates away 
     * from the current object (see prepareForLoad).
     */
    private Callable<Void> afterLoad = new Callable<Void>() {
        @Override
        public Void call() throws Exception {

            if (editor==null) {
                log.warn("Editor is null");
                return null;
            }
            
            DomainObjectEditorState<?,?,?> state = editor.saveState();
            if (state!=null) {
                state.setTopComponent(DomainListViewTopComponent.this);
                DataBrowserMgr.getDataBrowserMgr().getNavigationHistory(DomainListViewTopComponent.this).pushHistory(state);
            }
            else {
                log.warn("Editor did not provide current state");
            }
            
            // Update the editor name
            setName(editor.getName());

            return null;
        }
    };
    
    private boolean prepareForLoad(DomainObject domainObject) {

        // Can view display this object?
        final Class<? extends ParentNodeSelectionEditor<? extends DomainObject,?,?>> editorClass = getEditorClass(domainObject);
        if (editorClass==null) {
            return false;
        }

        // Do we already have the given node loaded?
        if (!setCurrent(domainObject)) {
            log.info("Domain object already loaded");
            return false;
        }

        // Update the previous editor state. Things may have changed since we saved it. 
        if (editor!=null) {
            DomainObjectEditorState<?,?,?> state = editor.saveState();
            if (state!=null) {
                state.setTopComponent(DomainListViewTopComponent.this);
                DataBrowserMgr.getDataBrowserMgr().getNavigationHistory(DomainListViewTopComponent.this).updateCurrentState(state);
            }
            else {
                log.warn("Editor did not provide current state");
            }
        }

        // Set the editor type
        if (editor==null || !editor.getClass().equals(editorClass)) {
            setEditorClass(editorClass);
        }
        
        // Reset the editor state
        editor.resetState();
        
        return true;
    }
    
    @SuppressWarnings("unchecked")
    public void loadState(DomainObjectEditorState<?,?,?> state) {

        log.trace("loadState({})", state);
        if (state==null) return;
        
        if (!prepareForLoad(state.getDomainObject())) return;
        editor.restoreState(state);

        ViewerUtils.updateContextIfChanged(this, content, editor.getViewerContext());
        ViewerUtils.updateNodeIfChanged(this, content, editor.getSelectionModel().getObjects());

        // TODO: this should run as a callback after loadState is fully complete
        // Update the editor name
        setName(editor.getName());
    }

    @SuppressWarnings("unchecked")
    private static Class<? extends ParentNodeSelectionEditor<? extends DomainObject,?,?>> getEditorClass(DomainObject domainObject) {
        DomainObjectHandler provider = ServiceAcceptorHelper.findFirstHelper(domainObject);
        if (provider!=null) {
            return (Class<? extends ParentNodeSelectionEditor<? extends DomainObject, ?, ?>>) 
                    provider.getEditorClass(domainObject);
        }
        return null;
    }
    
    public static boolean isSupported(DomainObject domainObject) {
        return getEditorClass(domainObject)!=null;
    }
}
