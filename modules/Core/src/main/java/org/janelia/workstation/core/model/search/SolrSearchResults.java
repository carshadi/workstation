package org.janelia.workstation.core.model.search;

/**
 * Search results backed by a SOLR search.
 * 
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public class SolrSearchResults extends DomainObjectSearchResults {

    private final SearchConfiguration searchConfig;
    
    public SolrSearchResults(SearchConfiguration searchConfig, DomainObjectResultPage firstPage) {
        super(firstPage);
        this.searchConfig = searchConfig;
    }

    @Override
    public DomainObjectResultPage getPage(int page) throws Exception {
        DomainObjectResultPage resultPage = super.getPage(page);
        if (resultPage==null) {
            resultPage = searchConfig.performSearch(page);
            setPage(page, resultPage);
        }
        return resultPage;
    }
}
