package org.janelia.workstation.browser.actions;

import org.openide.awt.ActionID;
import org.openide.awt.ActionReference;
import org.openide.awt.ActionReferences;
import org.openide.awt.ActionRegistration;
import org.openide.util.HelpCtx;
import org.openide.util.NbBundle.Messages;
import org.openide.util.actions.CallableSystemAction;

/**
 * Allows the user to create new filters, either in their default workspace, 
 * or underneath another existing tree node. Once the filter is created, 
 * it is opened in the filter editor.
 * 
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
@ActionID(
        category = "File",
        id = "org.janelia.workstation.browser.actions.NewFilterAction"
)
@ActionRegistration(
        displayName = "#CTL_NewFilterAction"
)
@ActionReferences({
        @ActionReference(path = "Menu/File/New", position = 1),
        @ActionReference(path = "Toolbars/Navigation", position = 1),
        @ActionReference(path = "Shortcuts", name = "D-S")
})
@Messages("CTL_NewFilterAction=Search")
public final class NewFilterAction extends CallableSystemAction {

    public NewFilterAction() {
    }

    @Override
    public String getName() {
        return "Search";
    }

    @Override
    protected String iconResource() {
        return "images/search-white-icon.png";
    }

    @Override
    public HelpCtx getHelpCtx() {
        return null;
    }

    @Override
    protected boolean asynchronous() {
        return false;
    }

    @Override
    public void performAction() {
        NewFilterActionListener actionListener = new NewFilterActionListener();
        actionListener.actionPerformed(null);
    }
}
