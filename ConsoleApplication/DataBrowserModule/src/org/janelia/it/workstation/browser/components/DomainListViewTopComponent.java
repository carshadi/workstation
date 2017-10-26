package org.janelia.it.workstation.browser.components;

import java.awt.BorderLayout;
import java.util.concurrent.Callable;

import javax.swing.JComponent;

import org.janelia.it.jacs.integration.FrameworkImplProvider;
import org.janelia.it.jacs.model.domain.DomainObject;
import org.janelia.it.jacs.model.domain.Reference;
import org.janelia.it.jacs.model.domain.gui.search.Filtering;
import org.janelia.it.jacs.model.domain.workspace.TreeNode;
import org.janelia.it.jacs.shared.utils.StringUtils;
import org.janelia.it.workstation.browser.ConsoleApp;
import org.janelia.it.workstation.browser.api.DomainMgr;
import org.janelia.it.workstation.browser.api.StateMgr;
import org.janelia.it.workstation.browser.events.Events;
import org.janelia.it.workstation.browser.gui.editor.DomainObjectEditorState;
import org.janelia.it.workstation.browser.gui.editor.DomainObjectNodeSelectionEditor;
import org.janelia.it.workstation.browser.gui.editor.FilterEditorPanel;
import org.janelia.it.workstation.browser.gui.editor.TreeNodeEditorPanel;
import org.janelia.it.workstation.browser.gui.find.FindContext;
import org.janelia.it.workstation.browser.gui.find.FindContextActivator;
import org.janelia.it.workstation.browser.gui.find.FindContextManager;
import org.janelia.it.workstation.browser.gui.support.MouseForwarder;
import org.janelia.it.workstation.browser.nodes.AbstractDomainObjectNode;
import org.janelia.it.workstation.browser.workers.SimpleWorker;
import org.netbeans.api.settings.ConvertAsProperties;
import org.openide.awt.ActionID;
import org.openide.awt.ActionReference;
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
        dtd = "-//org.janelia.it.workstation.browser.components//DomainListView//EN",
        autostore = false
)
@TopComponent.Description(
        preferredID = DomainListViewTopComponent.TC_NAME,
        //iconBase = "images/folder_open_page.png",
        persistenceType = TopComponent.PERSISTENCE_ONLY_OPENED
)
@TopComponent.Registration(mode = "editor", openAtStartup = false)
@ActionID(category = "Window", id = "org.janelia.it.workstation.browser.components.DomainListViewTopComponent")
@ActionReference(path = "Menu/Window/Core", position = 3)
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
    private DomainObjectNodeSelectionEditor editor;
    private FindContext findContext;
    private boolean active = false;
    
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
        DomainListViewManager.getInstance().activate(this);
    }
    
    @Override
    public void componentClosed() {
    }

    @Override
    protected void componentActivated() {
        log.info("Activating domain browser");
        this.active = true;
        // Make this the active list viewer
        DomainListViewManager.getInstance().activate(this);
        // Take control of the history navigation buttons
        StateMgr.getStateMgr().updateNavigationButtons(this);
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
            editor.activate();
        }
    }
    
    @Override
    protected void componentDeactivated() {
        this.active = false;
        if (findContext!=null) {
            FindContextManager.getInstance().deactivateContext(findContext);
        }
        if (editor!=null) {
            editor.deactivate();
        }
    }

    void writeProperties(java.util.Properties p) {
        if (p==null || editor==null) return;
        try {
            DomainObjectEditorState<?> state = editor.saveState();
            String serializedState = DomainObjectEditorState.serialize(state);
            log.info("Writing state: {}",serializedState);
            p.setProperty("version", TC_VERSION);
            p.setProperty("editorState", serializedState);
        }
        catch (Exception e) {
            FrameworkImplProvider.handleExceptionQuietly("Could not serialize editor state", e);
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

                SimpleWorker worker = new SimpleWorker() {
                    DomainObjectEditorState<?> state;
                    DomainObject domainObject;
                    
                    @Override
                    protected void doStuff() throws Exception {
                        state = DomainObjectEditorState.deserialize(editorState);
                        if (state.getDomainObject().getId()!=null) {
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
        }
        else if (TC_VERSION_IDONLY.equals(version)) {
            // An older version only saved the object id 
            final String objectStrRef = p.getProperty("objectRef");

            if (!StringUtils.isEmpty(objectStrRef)) {
                log.info("Reading state: {}",objectStrRef);
                
                SimpleWorker worker = new SimpleWorker() {
                    DomainObject domainObject;
                    
                    @Override
                    protected void doStuff() throws Exception {
                        domainObject = DomainMgr.getDomainMgr().getModel().getDomainObject(Reference.createFor(objectStrRef));
                    }

                    @Override
                    protected void hadSuccess() {
                        if (domainObject!=null) {
                            loadDomainObject(domainObject, false);
                        }
                    }

                    @Override
                    protected void hadError(Throwable error) {
                        FrameworkImplProvider.handleExceptionQuietly("Could not load serialized editor state",error);
                    }
                };
                worker.execute();
            }
        }
        else {
            log.warn("Unrecognized TC version: {}",version);
        }
    }
    
    // Custom methods

    @Override
    public void setFindContext(FindContext findContext) {
        this.findContext = findContext; 
        if (active) {
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
    
    public void setEditorClass(Class<? extends DomainObjectNodeSelectionEditor<?>> editorClass) {
        try {
            
            if (editor!=null) {
                remove((JComponent)editor);
                Events.getInstance().unregisterOnEventBus(editor);
                Events.getInstance().unregisterOnEventBus(editor.getEventBusListener());
            }
            
            editor = editorClass.newInstance();
            Events.getInstance().registerOnEventBus(editor.getEventBusListener());
            Events.getInstance().registerOnEventBus(editor);
            
            JComponent editorComponent = (JComponent)editor;
            editorComponent.addMouseListener(new MouseForwarder(this, "DomainObjectSelectionEditor->DomainListViewTopComponent"));
            add(editorComponent, BorderLayout.CENTER);
        }
        catch (InstantiationException | IllegalAccessException e) {
            ConsoleApp.handleException(e);
        }
        setName(editor.getName());
    }
    
    public DomainObjectNodeSelectionEditor<?> getEditor() {
        return editor;
    }

    @SuppressWarnings({ "unchecked" })
    public void loadDomainObjectNode(AbstractDomainObjectNode<?> domainObjectNode, boolean isUserDriven) {
        
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

            DomainObjectEditorState<?> state = editor.saveState();
            if (state!=null) {
                state.setTopComponent(DomainListViewTopComponent.this);
                StateMgr.getStateMgr().getNavigationHistory(DomainListViewTopComponent.this).pushHistory(state);
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
        final Class<? extends DomainObjectNodeSelectionEditor<?>> editorClass = getEditorClass(domainObject);
        if (editorClass==null) {
            return false;
        }

        // Do we already have the given node loaded?
        if (!setCurrent(domainObject)) {
            return false;
        }

        // Update the previous editor state. Things may have changed since we saved it. 
        if (editor!=null) {
            DomainObjectEditorState<?> state = editor.saveState();
            if (state!=null) {
                state.setTopComponent(DomainListViewTopComponent.this);
                StateMgr.getStateMgr().getNavigationHistory(DomainListViewTopComponent.this).updateCurrentState(state);
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
    public void loadState(DomainObjectEditorState<?> state) {

        log.trace("loadState({})", state);
        
        if (!prepareForLoad(state.getDomainObject())) return;
        editor.restoreState(state);
        
        // TODO: this should run as a callback after loadState is fully complete
        // Update the editor name
        setName(editor.getName());
    }

    private static Class<? extends DomainObjectNodeSelectionEditor<?>> getEditorClass(DomainObject domainObject) {
        if (TreeNode.class.isAssignableFrom(domainObject.getClass())) {
            return TreeNodeEditorPanel.class;
        }
        else if (Filtering.class.isAssignableFrom(domainObject.getClass())) {
            return FilterEditorPanel.class;
        }
        return null;
    }
    
    public static boolean isSupported(DomainObject domainObject) {
        return getEditorClass(domainObject)!=null;
    }
}
