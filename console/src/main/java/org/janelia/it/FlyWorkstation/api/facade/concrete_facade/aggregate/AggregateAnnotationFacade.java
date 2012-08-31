package org.janelia.it.FlyWorkstation.api.facade.concrete_facade.aggregate;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.janelia.it.FlyWorkstation.api.facade.abstract_facade.AnnotationFacade;
import org.janelia.it.jacs.model.entity.Entity;
import org.janelia.it.jacs.model.entity.EntityConstants;
import org.janelia.it.jacs.shared.annotation.DataDescriptor;
import org.janelia.it.jacs.shared.annotation.DataFilter;
import org.janelia.it.jacs.shared.annotation.FilterResult;
import org.janelia.it.jacs.shared.annotation.PatternAnnotationDataManager;

/**
 * Created by IntelliJ IDEA.
 * User: saffordt
 * Date: 7/22/11
 * Time: 4:50 PM
 */
public class AggregateAnnotationFacade extends AggregateEntityFacade implements AnnotationFacade {

    static private Object[] parameters = new Object[]{EntityConstants.TYPE_ANNOTATION};

    protected String getMethodNameForAggregates() {
        return ("getFacade");
    }

    protected Class[] getParameterTypesForAggregates() {
        return new Class[]{String.class};
    }

    protected Object[] getParametersForAggregates() {
        return parameters;
    }

    @Override
    public List<Entity> getAnnotationsForEntity(Long entityId) throws Exception {
        Object[] aggregates = getAggregates();
        List<Entity> returnList = new ArrayList<Entity>();
        List<Entity> tmpAnnotations;
        for (Object aggregate : aggregates) {
            tmpAnnotations = ((AnnotationFacade) aggregate).getAnnotationsForEntity(entityId);
            if (null != tmpAnnotations) {
                returnList.addAll(tmpAnnotations);
            }
        }
        return returnList;
    }

    @Override
    public List<Entity> getAnnotationsForEntities(List<Long> entityIds) throws Exception {
        Object[] aggregates = getAggregates();
        List<Entity> returnList = new ArrayList<Entity>();
        List<Entity> tmpAnnotations;
        for (Object aggregate : aggregates) {
            tmpAnnotations = ((AnnotationFacade) aggregate).getAnnotationsForEntities(entityIds);
            if (null != tmpAnnotations) {
                returnList.addAll(tmpAnnotations);
            }
        }
        return returnList;
    }

    @Override
    public List<Entity> getAnnotationsForChildren(Long parentId) throws Exception {
		throw new UnsupportedOperationException();
    }
    
    @Override
    public List<Entity> getEntitiesForAnnotationSession(Long annotationSessionId) throws Exception {
        Object[] aggregates = getAggregates();
        List<Entity> returnList = new ArrayList<Entity>();
        List<Entity> tmpEntities;
        for (Object aggregate : aggregates) {
            tmpEntities = ((AnnotationFacade) aggregate).getEntitiesForAnnotationSession(annotationSessionId);
            if (null != tmpEntities) {
                returnList.addAll(tmpEntities);
            }
        }
        return returnList;
    }

    @Override
    public List<Entity> getAnnotationsForSession(Long annotationSessionId) throws Exception {
        Object[] aggregates = getAggregates();
        List<Entity> returnList = new ArrayList<Entity>();
        List<Entity> tmpEntities;
        for (Object aggregate : aggregates) {
            tmpEntities = ((AnnotationFacade) aggregate).getAnnotationsForSession(annotationSessionId);
            if (null != tmpEntities) {
                returnList.addAll(tmpEntities);
            }
        }
        return returnList;
    }

    @Override
    public List<Entity> getCategoriesForAnnotationSession(Long annotationSessionId) throws Exception {
        Object[] aggregates = getAggregates();
        List<Entity> returnList = new ArrayList<Entity>();
        List<Entity> tmpCategories;
        for (Object aggregate : aggregates) {
            tmpCategories = ((AnnotationFacade) aggregate).getCategoriesForAnnotationSession(annotationSessionId);
            if (null != tmpCategories) {
                returnList.addAll(tmpCategories);
            }
        }
        return returnList;
    }

    @Override
    public Set<Long> getCompletedEntityIds(Long annotationSessionId) throws Exception {
		throw new UnsupportedOperationException();
    }
    
    @Override
    public void removeAnnotation(Long annotationId) throws Exception {
        Object[] aggregates = getAggregates();
        for (Object aggregate : aggregates) {
            ((AnnotationFacade) aggregate).removeAnnotation(annotationId);
        }
    }

    @Override
    public void removeAllOntologyAnnotationsForSession(Long annotationSessionId) throws Exception {
        Object[] aggregates = getAggregates();
        for (Object aggregate : aggregates) {
            ((AnnotationFacade) aggregate).removeAllOntologyAnnotationsForSession(annotationSessionId);
        }
    }
	
    @Override
    public Object[] getPatternAnnotationQuantifierMapsFromSummary() throws Exception {
        throw new UnsupportedOperationException();
    }

    @Override
    public Object[] getMaskQuantifierMapsFromSummary(String maskFolderName) throws Exception {
        throw new UnsupportedOperationException();
    }

    @Override
    public List<DataDescriptor> patternSearchGetDataDescriptors(String type) throws Exception {
        throw new UnsupportedOperationException();
    }

    @Override
    public int patternSearchGetState() throws Exception {
        throw new UnsupportedOperationException();
    }

    @Override
    public List<String> patternSearchGetCompartmentList(String type) throws Exception {
        throw new UnsupportedOperationException();
    }

    @Override
    public FilterResult patternSearchGetFilteredResults(String type, Map<DataDescriptor, Set<DataFilter>> filterMap) throws Exception {
        throw new UnsupportedOperationException();
    }


}
