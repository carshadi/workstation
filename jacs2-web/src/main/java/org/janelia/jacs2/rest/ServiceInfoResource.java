package org.janelia.jacs2.rest;

import org.janelia.jacs2.model.page.PageResult;
import org.janelia.jacs2.model.service.JacsServiceData;
import org.janelia.jacs2.model.service.ServiceMetaData;
import org.janelia.jacs2.service.JacsServiceDataManager;
import org.janelia.jacs2.service.ServiceRegistry;

import javax.enterprise.context.ApplicationScoped;
import javax.inject.Inject;
import javax.ws.rs.GET;
import javax.ws.rs.Path;
import javax.ws.rs.PathParam;
import javax.ws.rs.Produces;
import javax.ws.rs.QueryParam;
import javax.ws.rs.core.Response;

@ApplicationScoped
@Produces("application/json")
@Path("/v2/services")
public class ServiceInfoResource {

    @Inject
    private JacsServiceDataManager jacsServiceDataManager;
    @Inject
    private ServiceRegistry serviceRegistry;

    @GET
    public PageResult<JacsServiceData> getAllServices(@QueryParam("service-name") String serviceName,
                                               @QueryParam("service-state") String serviceState,
                                               @QueryParam("page") Integer pageNumber,
                                               @QueryParam("length") Integer pageLength) {
        System.out.println("!!!! NAME QUERY PARAM " + serviceName);
        System.out.println("!!!! STATE QUERY PARAM " + serviceState);
        return new PageResult<>(); // TODO implement me
    }

    @GET
    @Path("/{service-instance-id}")
    public JacsServiceData getServiceInfo(@PathParam("service-instance-id") long instanceId) {
        return jacsServiceDataManager.retrieveServiceData(instanceId);
    }

    @GET
    @Path("/{service-name}/metadata")
    public Response getServiceMetadata(@PathParam("service-name") String serviceName) {
        System.out.println("!!!!!! METADATA " + serviceName);
        ServiceMetaData smd = serviceRegistry.getServiceDescriptor(serviceName);
        if (smd == null) {
            return Response
                    .status(Response.Status.NOT_FOUND)
                    .build();
        }
        return Response
                .status(Response.Status.OK)
                .entity(smd)
                .build();
    }

}