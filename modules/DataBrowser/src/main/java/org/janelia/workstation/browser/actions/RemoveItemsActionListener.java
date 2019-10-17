package org.janelia.workstation.browser.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import javax.swing.JOptionPane;
import javax.swing.ProgressMonitor;

import com.google.common.collect.ArrayListMultimap;
import com.google.common.collect.Multimap;
import org.apache.commons.lang3.StringUtils;
import org.janelia.model.domain.DomainObject;
import org.janelia.model.domain.Reference;
import org.janelia.model.domain.workspace.Node;
import org.janelia.model.security.util.SubjectUtils;
import org.janelia.workstation.core.activity_logging.ActivityLogHelper;
import org.janelia.workstation.core.api.AccessManager;
import org.janelia.workstation.core.api.DomainMgr;
import org.janelia.workstation.core.api.DomainModel;
import org.janelia.workstation.core.workers.SimpleWorker;
import org.janelia.workstation.integration.spi.domain.DomainObjectHandler;
import org.janelia.workstation.integration.spi.domain.ServiceAcceptorHelper;
import org.janelia.workstation.integration.util.FrameworkAccess;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Remove items, either from a node, or from the system entirely.
 * 
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public class RemoveItemsActionListener implements ActionListener {

    private final static Logger log = LoggerFactory.getLogger(RemoveItemsActionListener.class);

    private final Node node;
    private final Collection<DomainObject> domainObjects;

    public RemoveItemsActionListener(Node node, Collection<DomainObject> domainObjects) {
        this.node = node;
        this.domainObjects = domainObjects;
    }

    @Override
    public void actionPerformed(ActionEvent event) {
    	
        ActivityLogHelper.logUserAction("RemoveItemsActionListener.doAction", node);

        final DomainModel model = DomainMgr.getDomainMgr().getModel();
        final Multimap<Node,DomainObject> removeFromFolders = ArrayListMultimap.create();
        final List<DomainObject> listToDelete = new ArrayList<>();

        for(DomainObject domainObject : domainObjects) {

            DomainObjectHandler provider = ServiceAcceptorHelper.findFirstHelper(domainObject);
            if (provider!=null && provider.supportsRemoval(domainObject)) {
                if (node==null) {
                    listToDelete.add(domainObject);
                }
                else {
                    // first check to make sure this Object only has one ancestor references; if it does pop up a dialog before removal
                    List<Reference> refList = model.getContainerReferences(domainObject);
                    if (refList==null || refList.size()<=provider.getMaxReferencesBeforeRemoval(domainObject)) {
                        listToDelete.add(domainObject);
                    }
                    else {
                        log.info("{} has multiple references: {}", domainObject, refList);
                    }
                }
            }
            else {
                log.trace("Removal not supported for {}", domainObject);
            }

            if (node!=null) {
                log.info("Will remove {} from {}", domainObject, node);
                removeFromFolders.put(node,domainObject);
            }
        }
        
        if (!listToDelete.isEmpty()) {
            
            Set<String> allReaders = new HashSet<>();
            for (DomainObject domainObject : listToDelete) {
                allReaders.addAll(domainObject.getReaders());
                allReaders.remove(domainObject.getOwnerKey());
            }
            allReaders.remove(AccessManager.getSubjectKey());
            
            if (!allReaders.isEmpty()) {
                List<String> sharedWith = allReaders.stream().map(SubjectUtils::getSubjectName).collect(Collectors.toList());    
                String sharedWithList = StringUtils.join(sharedWith, ", ");
                
                int deleteConfirmation = JOptionPane.showConfirmDialog(FrameworkAccess.getMainFrame(),
                        "There are " + listToDelete.size() + " items in your remove list that will be deleted permanently. These items are shared with: "+sharedWithList+". Delete anyway?",
                        "Are you sure?", JOptionPane.YES_NO_OPTION);
                if (deleteConfirmation != 0) {
                    return;
                }
            }
            else {
                int deleteConfirmation = JOptionPane.showConfirmDialog(FrameworkAccess.getMainFrame(),
                        "There are " + listToDelete.size() + " items in your remove list that will be deleted permanently.",
                        "Are you sure?", JOptionPane.YES_NO_OPTION);
                if (deleteConfirmation != 0) {
                    return;
                }
            }
        }

        SimpleWorker worker = new SimpleWorker() {

            @Override
            protected void doStuff() throws Exception {

                // Remove any actual objects that are no longer referenced
                if (!listToDelete.isEmpty()) {
                    log.debug("Looking for provider to deleting object entirely: {}", listToDelete);
                    for(DomainObject domainObject : listToDelete) {
                        DomainObjectHandler provider = ServiceAcceptorHelper.findFirstHelper(domainObject);
                        if (provider!=null) {
                            log.info("Using {} to delete object {}", provider.getClass().getName(), domainObject);
                            provider.remove(domainObject);
                        }
                        else {
                            log.warn("No DomainObjectHandler found for {}, cannot delete.",domainObject);
                        }
                    }
                }
                
                // Delete references
                for (Node node : removeFromFolders.keySet()) {
                    Collection<DomainObject> items = removeFromFolders.get(node);
                    log.info("Removing {} items from {}", items.size(), node);
                    model.removeChildren(node, items);
                }
            }

            @Override
            protected void hadSuccess() {
                // Handled by the event system
            }

            @Override
            protected void hadError(Throwable error) {
                FrameworkAccess.handleException(error);
            }
        };

        worker.setProgressMonitor(new ProgressMonitor(FrameworkAccess.getMainFrame(), "Removing items", "", 0, 100));
        worker.execute();
    }
}
