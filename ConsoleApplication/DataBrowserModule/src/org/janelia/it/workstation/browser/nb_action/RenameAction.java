package org.janelia.it.workstation.browser.nb_action;

import java.util.ArrayList;
import java.util.List;

import javax.swing.JOptionPane;

import org.janelia.it.jacs.model.domain.DomainObject;
import org.janelia.it.workstation.browser.ConsoleApp;
import org.janelia.it.workstation.browser.api.ClientDomainUtils;
import org.janelia.it.workstation.browser.api.DomainMgr;
import org.janelia.it.workstation.browser.api.DomainModel;
import org.janelia.it.workstation.browser.nodes.DomainObjectNode;
import org.openide.nodes.Node;
import org.openide.util.HelpCtx;
import org.openide.util.actions.NodeAction;

/**
 * Action which implements rename for domain objects represented as nodes. 
 * 
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public final class RenameAction extends NodeAction {

    private final static RenameAction singleton = new RenameAction();
    public static RenameAction get() {
        return singleton;
    }

    private final List<Node> selected = new ArrayList<>();
    private List<DomainObjectNode<DomainObject>> toRename = new ArrayList<>();

    public RenameAction() {
        putValue(NAME, "Rename");
    }

    @Override
    public String getName() {
        return "Rename "+toRename.size()+" items";
    }

    @Override
    public HelpCtx getHelpCtx() {
        return new HelpCtx("RemoveAction");
    }
    
    @Override
    protected boolean asynchronous() {
        return false;
    }

    @Override
    protected boolean enable(Node[] activatedNodes) {
        selected.clear();
        toRename.clear();
        for(Node node : activatedNodes) {
            selected.add(node);
            
            if (node instanceof DomainObjectNode) {
                DomainObjectNode<DomainObject> domainObjectNode = (DomainObjectNode<DomainObject>)node;
                DomainObject domainObject = domainObjectNode.getDomainObject();
                if (ClientDomainUtils.hasWriteAccess(domainObject)) {
                    toRename.add(domainObjectNode);
                }
            }
        }
        
        return selected.size()==1 && toRename.size()==selected.size();
    }
    
    @Override
    protected void performAction (Node[] activatedNodes) {
        DomainObjectNode<DomainObject> domainObjectNode = toRename.get(0);
        DomainObject domainObject = domainObjectNode.getDomainObject();
        String newName = (String) JOptionPane.showInputDialog(ConsoleApp.getMainFrame(), "Name:\n", "Rename "
                + domainObject.getName(), JOptionPane.PLAIN_MESSAGE, null, null, domainObject.getName());
        if ((newName == null) || (newName.length() <= 0)) {
            return;
        }
        try {
            DomainModel model = DomainMgr.getDomainMgr().getModel();
            DomainObject updated = model.updateProperty(domainObject, "name", newName);
            domainObjectNode.update(updated); 
        } 
        catch (Exception ex) {
            ConsoleApp.handleException(ex);
        }
    }
}