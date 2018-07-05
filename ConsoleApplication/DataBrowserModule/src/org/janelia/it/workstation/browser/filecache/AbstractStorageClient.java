package org.janelia.it.workstation.browser.filecache;

import java.io.FileNotFoundException;
import java.net.MalformedURLException;
import java.net.URL;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.apache.commons.httpclient.Header;
import org.apache.commons.httpclient.HttpClient;
import org.apache.http.HttpStatus;
import org.apache.jackrabbit.webdav.MultiStatus;
import org.apache.jackrabbit.webdav.MultiStatusResponse;
import org.apache.jackrabbit.webdav.client.methods.PropFindMethod;
import org.janelia.it.workstation.browser.api.http.HttpClientProxy;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * {@link HttpClient} wrapper for submitting WebDAV requests.
 *
 * @author Eric Trautman
 */
abstract class AbstractStorageClient {

    private static final Logger LOG = LoggerFactory.getLogger(AbstractStorageClient.class);

    final String baseUrl;
    final HttpClientProxy httpClient;
    final ObjectMapper objectMapper;

    /**
     * Constructs a client with default authentication credentials.
     *
     * @param  httpClient             httpClient
     *                                (e.g. /groups/...) to WebDAV URLs.
     * @throws IllegalArgumentException
     *   if the baseUrl cannot be parsed.
     */
    AbstractStorageClient(String baseUrl, HttpClientProxy httpClient, ObjectMapper objectMapper) {
        this.baseUrl = validateUrl(baseUrl);
        this.httpClient = httpClient;
        this.objectMapper = objectMapper;
    }

    private String validateUrl(String urlString) {
        try {
            final URL url = new URL(urlString);
            return urlString;
        } catch (MalformedURLException e) {
            throw new IllegalArgumentException("failed to parse URL: " + urlString, e);
        }
    }

    MultiStatusResponse[] getResponses(String href, int depth, int callCount)
            throws WebDavException {
        MultiStatusResponse[] multiStatusResponses;
        PropFindMethod method = null;
        try {
            method = new PropFindMethod(href, WebDavFile.PROPERTY_NAMES, depth);
            method.addRequestHeader("Accept", "application/xml");
            method.addRequestHeader("Content-Type", "application/xml");
            final int responseCode = httpClient.executeMethod(method);
            LOG.trace("getResponses: {} returned for PROPFIND {}", responseCode, href);

            if (responseCode == HttpStatus.SC_MULTI_STATUS) {
                final MultiStatus multiStatus = method.getResponseBodyAsMultiStatus();
                multiStatusResponses = multiStatus.getResponses();
            } else if (responseCode == HttpStatus.SC_MOVED_PERMANENTLY) {
                final Header locationHeader = method.getResponseHeader("Location");
                if (locationHeader != null) {
                    final String movedHref = locationHeader.getValue();
                    if (callCount == 0) {
                        return getResponses(movedHref, depth, 1);
                    }
                }
                throw new WebDavException(responseCode + " response code returned for " + href, responseCode);
            } else if (responseCode == HttpStatus.SC_NOT_FOUND) {
                throw new FileNotFoundException("Resource " + href + "not found (" + responseCode + ")");
            } else {
                throw new WebDavException(responseCode + " response code returned for " + href, responseCode);
            }
        } catch (WebDavException e) {
            throw e;
        } catch (Exception e) {
            throw new WebDavException("failed to retrieve WebDAV information from " + href, e);
        } finally {
            if (method != null) {
                method.releaseConnection();
            }
        }
        return multiStatusResponses;
    }

    String getStorageLookupURL(String remoteFileName, String context) {
        return baseUrl + "/" + context + "/" + remoteFileName;
    }

}
