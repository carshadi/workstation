package org.janelia.jacs2.cdi;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import org.apache.commons.lang3.StringUtils;
import org.janelia.jacs2.cdi.qualifier.ApplicationProperties;
import org.janelia.jacs2.cdi.qualifier.PropertyValue;
import org.janelia.jacs2.utils.TimebasedIdentifierGenerator;

import javax.enterprise.context.ApplicationScoped;
import javax.enterprise.inject.Produces;
import javax.enterprise.inject.spi.InjectionPoint;
import javax.inject.Singleton;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

public class ApplicationProducer {

    @Singleton
    @Produces
    public ObjectMapper objectMapper() {
        return ObjectMapperFactory.instance().getObjectMapper();
    }

    @Produces
    public TimebasedIdentifierGenerator idGenerator(@PropertyValue(name = "TimebasedIdentifierGenerator.DeploymentContext") Integer deploymentContext) {
        return new TimebasedIdentifierGenerator(deploymentContext);
    }

    @Produces
    @PropertyValue(name = "")
    public String stringPropertyValue(@ApplicationProperties Properties properties, InjectionPoint injectionPoint) {
        final PropertyValue property = injectionPoint.getAnnotated().getAnnotation(PropertyValue.class);
        return properties.getProperty(property.name());
    }

    @Produces
    @PropertyValue(name = "")
    public Integer integerPropertyValue(@ApplicationProperties Properties properties, InjectionPoint injectionPoint) {
        String stringValue = stringPropertyValue(properties, injectionPoint);
        return stringValue == null ? null : Integer.valueOf(stringValue);
    }

    @ApplicationScoped
    @ApplicationProperties
    @Produces
    public Properties properties() throws IOException {
        Properties appProperties = new Properties();
        try (InputStream configReader = PersistenceProducer.class.getResourceAsStream("/jacs.properties")) {
            appProperties.load(configReader);
        }
        String jacs2ConfigEnv = System.getenv("JACS2_CONFIG");
        if (StringUtils.isBlank(jacs2ConfigEnv)) {
            return appProperties;
        }
        File jacs2ConfigFile = new File(jacs2ConfigEnv);
        if (jacs2ConfigFile.exists() && jacs2ConfigFile.isFile()) {
            try (InputStream configReader = new FileInputStream(jacs2ConfigFile)) {
                appProperties.load(configReader);
            }
        }
        return appProperties;
    }

}
