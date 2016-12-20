package org.janelia.it.workstation.gui.geometric_search.viewer.event;

import org.janelia.it.workstation.gui.geometric_search.viewer.dataset.Dataset;

/**
 * Created by murphys on 8/20/2015.
 */
public class DatasetSelectedEvent extends VoxelViewerEvent {

    Dataset dataset;

    public DatasetSelectedEvent(Dataset dataset) {
        this.dataset = dataset;
    }

    public Dataset getDataset() {
        return dataset;
    }

    public void setDataset(Dataset dataset) {
        this.dataset = dataset;
    }
}