package org.janelia.workstation.browser.model;

import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import com.fasterxml.jackson.annotation.JsonIgnore;
import org.janelia.model.domain.gui.cdmip.ColorDepthImage;
import org.janelia.model.domain.gui.cdmip.ColorDepthLibrary;
import org.janelia.model.domain.gui.search.Filtering;
import org.janelia.model.domain.gui.search.criteria.AttributeValueCriteria;
import org.janelia.model.domain.gui.search.criteria.Criteria;
import org.janelia.model.domain.gui.search.criteria.FacetCriteria;

/**
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public class ColorDepthAlignmentSpace implements Filtering {

    private ColorDepthLibrary library;
    private String alignmentSpace;
    private List<Criteria> lazyCriteria;
    private int index;

    // For deserialization
    public ColorDepthAlignmentSpace() {
    }

    public ColorDepthAlignmentSpace(ColorDepthLibrary library, String alignmentSpace, int index) {
        this.library = library;
        this.alignmentSpace = alignmentSpace;
        this.index = index;
    }

    public ColorDepthLibrary getLibrary() {
        return library;
    }

    public String getAlignmentSpace() {
        return alignmentSpace;
    }

    public int getIndex() {
        return index;
    }

    @Override
    public Long getId() {
        return null;
    }

    @Override
    public void setId(Long id) {
        throw new UnsupportedOperationException();
    }

    @Override
    public String getName() {
        return library.getName()+" "+alignmentSpace;
    }

    @Override
    public void setName(String name) {
        throw new UnsupportedOperationException();
    }

    @Override
    public String getOwnerKey() {
        return library.getOwnerKey();
    }

    @Override
    public void setOwnerKey(String ownerKey) {
        throw new UnsupportedOperationException();
    }

    @Override
    public Set<String> getReaders() {
        return library.getReaders();
    }

    @Override
    public void setReaders(Set<String> readers) {
        throw new UnsupportedOperationException();
    }

    @Override
    public Set<String> getWriters() {
        return library.getWriters();
    }

    @Override
    public void setWriters(Set<String> writers) {
        throw new UnsupportedOperationException();
    }

    @Override
    public Date getCreationDate() {
        return library.getCreationDate();
    }

    @Override
    public void setCreationDate(Date creationDate) {
        throw new UnsupportedOperationException();
    }

    @Override
    public Date getUpdatedDate() {
        return library.getUpdatedDate();
    }

    @Override
    public void setUpdatedDate(Date updatedDate) {
        throw new UnsupportedOperationException();
    }

    @Override
    public String getType() {
        return library.getType();
    }


    /* implement Filtering interface */

    @JsonIgnore
    @Override
    public String getSearchClass() {
        return ColorDepthImage.class.getName();
    }

    @JsonIgnore
    @Override
    public boolean hasCriteria() {
        return true;
    }

    @JsonIgnore
    @Override
    public String getSearchString() {
        return null;
    }

    @JsonIgnore
    @Override
    public List<Criteria> getCriteriaList() {
        if (lazyCriteria==null) {

            lazyCriteria = new ArrayList<>();

            // Search for images in this library
            FacetCriteria libraryIdentifier = new FacetCriteria();
            libraryIdentifier.setAttributeName("libraries");
            libraryIdentifier.getValues().add(library.getIdentifier());
            lazyCriteria.add(libraryIdentifier);

            FacetCriteria alignmentSpace = new FacetCriteria();
            alignmentSpace.setAttributeName("alignmentSpace");
            alignmentSpace.getValues().add(this.alignmentSpace);
            lazyCriteria.add(alignmentSpace);

        }
        return lazyCriteria;
    }

}
