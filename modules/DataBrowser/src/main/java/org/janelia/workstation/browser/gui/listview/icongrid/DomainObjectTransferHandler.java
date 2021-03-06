package org.janelia.workstation.browser.gui.listview.icongrid;

import java.awt.datatransfer.Transferable;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JComponent;
import javax.swing.TransferHandler;

import org.janelia.workstation.integration.util.FrameworkAccess;
import org.janelia.workstation.core.events.selection.DomainObjectSelectionModel;
import org.janelia.workstation.core.model.ImageModel;
import org.janelia.model.domain.DomainObject;
import org.janelia.model.domain.Reference;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Support for dragging domain objects and dropping them onto the Explorer.
 *
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public class DomainObjectTransferHandler extends TransferHandler {

    private static final Logger log = LoggerFactory.getLogger(DomainObjectTransferHandler.class);

    private final ImageModel<DomainObject,Reference> imageModel;
    private final DomainObjectSelectionModel selectionModel;
    
    public DomainObjectTransferHandler(ImageModel<DomainObject,Reference> imageModel, DomainObjectSelectionModel selectionModel) {
        this.imageModel = imageModel;
        this.selectionModel = selectionModel;
    }
    
    @Override
    protected Transferable createTransferable(JComponent sourceComponent) {
        try {
            log.debug("createTransferable sourceComponent={}", sourceComponent);

            if (sourceComponent instanceof AnnotatedImageButton) {
                List<DomainObject> domainObjects = new ArrayList<>();
                for(Reference id : selectionModel.getSelectedIds()) {
                    domainObjects.add(imageModel.getImageByUniqueId(id));
                }
                return new TransferableDomainObjectList(sourceComponent, domainObjects);
            }
            else {
                log.warn("Unsupported component type for transfer: " + sourceComponent.getClass().getName());
                return null;
            }
        }  catch (Exception e) {
            FrameworkAccess.handleException(e);
            return null;
        }
    }

    @Override
    public int getSourceActions(JComponent sourceComponent) {
        return LINK;
    }
    
    protected DomainObjectSelectionModel getSelectionModel() {
        return selectionModel;
    }
    
    protected ImageModel<DomainObject, Reference> getImageModel() {
        return imageModel;
    }
}
