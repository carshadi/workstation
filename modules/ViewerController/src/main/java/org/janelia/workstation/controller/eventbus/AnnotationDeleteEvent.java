package org.janelia.workstation.controller.eventbus;

import org.janelia.model.domain.tiledMicroscope.TmGeoAnnotation;
import java.util.Collection;

public class AnnotationDeleteEvent extends AnnotationEvent {
    public AnnotationDeleteEvent(Collection<TmGeoAnnotation> annotations) {
        this.annotations = annotations;
    }
}

