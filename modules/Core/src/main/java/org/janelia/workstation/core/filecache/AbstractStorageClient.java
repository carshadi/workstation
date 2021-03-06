package org.janelia.workstation.core.filecache;

import java.io.UnsupportedEncodingException;
import java.net.MalformedURLException;
import java.net.URL;
import java.net.URLEncoder;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.apache.commons.httpclient.HttpClient;
import org.janelia.workstation.core.api.http.HttpClientProxy;

/**
 * {@link HttpClient} wrapper for submitting WebDAV requests.
 *
 * @author Eric Trautman
 */
abstract class AbstractStorageClient {

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

    String getUrlEncodedPath(String p) {
        try {
            return URLEncoder.encode(p, "UTF-8");
        } catch (UnsupportedEncodingException e) {
            throw new IllegalArgumentException(e);
        }
    }

    private String validateUrl(String urlString) {
        try {
            new URL(urlString); // attempt to create URL only for validation purposes
            return urlString;
        } catch (MalformedURLException e) {
            throw new IllegalArgumentException("failed to parse URL: " + urlString, e);
        }
    }

    String getBaseUrl() {
        return baseUrl;
    }
}
