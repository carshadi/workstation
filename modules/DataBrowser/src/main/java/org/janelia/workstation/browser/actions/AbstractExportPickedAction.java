package org.janelia.workstation.browser.actions;

import java.awt.event.ActionEvent;
import java.io.File;
import java.io.FileWriter;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import javax.swing.AbstractAction;
import javax.swing.JOptionPane;

import org.janelia.model.domain.DomainObject;
import org.janelia.model.domain.Reference;
import org.janelia.workstation.core.activity_logging.ActivityLogHelper;
import org.janelia.workstation.core.api.DomainMgr;
import org.janelia.workstation.core.util.Utils;
import org.janelia.workstation.core.workers.IndeterminateProgressMonitor;
import org.janelia.workstation.core.workers.SimpleWorker;
import org.janelia.workstation.integration.util.FrameworkAccess;

/**
 * Export the picked results.
 * 
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public abstract class AbstractExportPickedAction extends AbstractAction {

    protected static final String DEFAULT_EXPORT_DIR = System.getProperty("java.io.tmpdir");
    private static final String NEW_LINE = System.getProperty("line.separator");

    private List<Reference> refs;

    public AbstractExportPickedAction(List<Reference> refs) {
        this.refs = refs;
    }
    
    @Override
    public void actionPerformed(ActionEvent event) {

        if (refs.isEmpty()) {
            JOptionPane.showMessageDialog(FrameworkAccess.getMainFrame(),
                    "Select some items with the checkboxes first.", 
                    "No items picked for export", JOptionPane.ERROR_MESSAGE);
            return;
        }
        
        ActivityLogHelper.logUserAction("AbstractExportPickedAction.doAction");

        File destFile = Utils.getOutputFile(DEFAULT_EXPORT_DIR, "WorkstationExport", "txt");

        SimpleWorker worker = new SimpleWorker() {

            @Override
            protected void doStuff() throws Exception {
                export(destFile);
            }

            @Override
            protected void hadSuccess() {
                OpenWithDefaultAppAction action = new OpenWithDefaultAppAction(destFile.getAbsolutePath());
                action.actionPerformed(null);
            }

            @Override
            protected void hadError(Throwable error) {
                FrameworkAccess.handleException(error);
            }
        };

        worker.setProgressMonitor(new IndeterminateProgressMonitor(FrameworkAccess.getMainFrame(), "Exporting data...", ""));
        worker.execute();
    }
    
    private void export(File destFile) throws Exception {

        Set<String> exports = new LinkedHashSet<>();
        for (DomainObject domainObject : DomainMgr.getDomainMgr().getModel().getDomainObjects(refs)) {
            exports.add(export(domainObject));
        }
        
        try (FileWriter writer = new FileWriter(destFile)) {
            StringBuilder buf = new StringBuilder();
            for (String guid : exports) {
                buf.append(guid);
                buf.append(NEW_LINE);
            }
            writer.write(buf.toString());
        }
    }

    protected abstract String export(DomainObject domainObject);
}
