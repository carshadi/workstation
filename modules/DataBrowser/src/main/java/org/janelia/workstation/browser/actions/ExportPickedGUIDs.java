package org.janelia.workstation.browser.actions;

import java.util.List;

import org.janelia.model.domain.DomainObject;
import org.janelia.model.domain.Reference;

/**
 * Export the unique GUIDs for the picked results.
 * 
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public class ExportPickedGUIDs extends AbstractExportPickedAction {

    public ExportPickedGUIDs(List<Reference> refs) {
        super(refs);
    }

    @Override
    protected String export(DomainObject domainObject) {
        return domainObject.toString();
    }
}
