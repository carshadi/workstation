package org.janelia.console.viewerapi.listener;

import org.janelia.model.domain.tiledMicroscope.TmNeuronMetadata;

import java.util.Collection;

public interface NeuronUpdateListener {

    void neuronsUpdated(Collection<TmNeuronMetadata> createdNeurons);
    
}
