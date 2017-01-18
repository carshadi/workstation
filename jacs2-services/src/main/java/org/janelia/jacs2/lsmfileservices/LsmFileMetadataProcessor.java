package org.janelia.jacs2.lsmfileservices;

import com.beust.jcommander.JCommander;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableMap;
import org.apache.commons.lang3.StringUtils;
import org.janelia.jacs2.cdi.qualifier.PropertyValue;
import org.janelia.jacs2.model.service.JacsServiceData;
import org.janelia.jacs2.persistence.JacsServiceDataPersistence;
import org.janelia.jacs2.service.impl.AbstractExeBasedServiceProcessor;
import org.janelia.jacs2.service.impl.ComputationException;
import org.janelia.jacs2.service.impl.ExternalProcessRunner;
import org.janelia.jacs2.service.impl.JacsServiceDispatcher;
import org.janelia.jacs2.service.impl.ServiceComputation;
import org.janelia.jacs2.service.impl.ServiceComputationFactory;
import org.slf4j.Logger;

import javax.enterprise.inject.Any;
import javax.enterprise.inject.Instance;
import javax.inject.Inject;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.attribute.PosixFilePermission;
import java.nio.file.attribute.PosixFilePermissions;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class LsmFileMetadataProcessor extends AbstractExeBasedServiceProcessor<File> {

    private static final String PERLLIB_VARNAME = "PERL5LIB";

    private final String perlExecutable;
    private final String perlModule;
    private final String scriptName;

    @Inject
    public LsmFileMetadataProcessor(JacsServiceDispatcher jacsServiceDispatcher,
                                    ServiceComputationFactory computationFactory,
                                    JacsServiceDataPersistence jacsServiceDataPersistence,
                                    @PropertyValue(name = "service.DefaultWorkingDir") String defaultWorkingDir,
                                    @PropertyValue(name = "Executables.ModuleBase") String executablesBaseDir,
                                    @Any Instance<ExternalProcessRunner> serviceRunners,
                                    @PropertyValue(name = "Perl.Path") String perlExecutable,
                                    @PropertyValue(name = "Sage.Perllib") String perlModule,
                                    @PropertyValue(name = "LSMJSONDump.CMD") String scriptName,
                                    Logger logger) {
        super(jacsServiceDispatcher, computationFactory, jacsServiceDataPersistence, defaultWorkingDir, executablesBaseDir, serviceRunners, logger);
        this.perlExecutable = perlExecutable;
        this.perlModule = perlModule;
        this.scriptName = scriptName;
    }

    @Override
    public File getResult(JacsServiceData jacsServiceData) {
        if (StringUtils.isNotBlank(jacsServiceData.getStringifiedResult())) {
            return new File(jacsServiceData.getStringifiedResult());
        } else {
            return null;
        }
    }

    @Override
    public void setResult(File result, JacsServiceData jacsServiceData) {
        if (result != null) {
            jacsServiceData.setStringifiedResult(result.toString());
        } else {
            jacsServiceData.setStringifiedResult(null);
        }
    }

    @Override
    protected ServiceComputation<File> preProcessData(JacsServiceData jacsServiceData) {
        return computationFactory.newCompletedComputation(jacsServiceData)
                .thenApply(sd -> {
                    try {
                        LsmFileMetadataServiceDescriptor.LsmFileMetadataArgs args = getArgs(jacsServiceData);
                        if (StringUtils.isBlank(args.inputLSMFile)) {
                            throw new ComputationException(jacsServiceData, "Input LSM file name must be specified");
                        } else if (StringUtils.isBlank(args.outputLSMMetadata)) {
                            throw new ComputationException(jacsServiceData, "Output LSM metadata name must be specified");
                        }
                        return getOutputFile(args);
                    } catch (Exception e) {
                        logger.error("FileCopy preprocess error", e);
                        throw new ComputationException(jacsServiceData, e);
                    }
                });
    }

    @Override
    protected ServiceComputation<File> localProcessData(Object preProcessingResult, JacsServiceData jacsServiceData) {
        return invokeExternalProcess(jacsServiceData)
                .thenApply(r -> {
                    try {
                        File lsmMetadataFile = (File) preProcessingResult;
                        if (!lsmMetadataFile.exists()) {
                            throw new ComputationException(jacsServiceData, "LSM metadata file " + lsmMetadataFile + " was not created");
                        }
                        setResult(lsmMetadataFile, jacsServiceData);
                        return lsmMetadataFile;
                    } catch (Exception e) {
                        throw new ComputationException(jacsServiceData, e);
                    }
                });
    }

    @Override
    protected ServiceComputation<File> postProcessData(File processingResult, JacsServiceData jacsServiceData) {
        return computationFactory.newCompletedComputation(processingResult)
                .thenApply(f -> {
                    try {
                        logger.debug("Delete temporary service script: {}", jacsServiceData.getServiceCmd());
                        Files.deleteIfExists(new File(jacsServiceData.getServiceCmd()).toPath());
                        return f;
                    } catch (Exception e) {
                        throw new ComputationException(jacsServiceData, e);
                    }
                });
    }

    @Override
    protected List<String> prepareCmdArgs(JacsServiceData jacsServiceData) {
        LsmFileMetadataServiceDescriptor.LsmFileMetadataArgs args = getArgs(jacsServiceData);
        File scriptFile = createScript(jacsServiceData, args);
        jacsServiceData.setServiceCmd(scriptFile.getAbsolutePath());
        return ImmutableList.of();
    }

    private File createScript(JacsServiceData jacsServiceData, LsmFileMetadataServiceDescriptor.LsmFileMetadataArgs args) {
        File inputFile = getInputFile(args);
        File outputFile = getOutputFile(args);
        File workingDir = outputFile.getParentFile();
        Set<PosixFilePermission> perms = PosixFilePermissions.fromString("rwx------");
        BufferedWriter outputStream = null;
        File scriptFile = null;
        try {
            scriptFile = Files.createFile(
                    Paths.get(workingDir.getAbsolutePath(), jacsServiceData.getName() + "_" + jacsServiceData.getId() + ".sh"),
                    PosixFilePermissions.asFileAttribute(perms)).toFile();
            outputStream = new BufferedWriter(new FileWriter(scriptFile));
            outputStream.append(String.format("%s %s %s > %s\n", perlExecutable, getFullExecutableName(scriptName), inputFile.getAbsoluteFile(), outputFile.getAbsoluteFile()));
            outputStream.flush();
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        } finally {
            if (outputStream != null) {
                try {
                    outputStream.close();
                } catch (IOException ignore) {
                }
            }
        }
        return scriptFile;
    }

    @Override
    protected Map<String, String> prepareEnvironment(JacsServiceData jacsServiceData) {
        return ImmutableMap.of(
            PERLLIB_VARNAME, perlModule
        );
    }

    private LsmFileMetadataServiceDescriptor.LsmFileMetadataArgs getArgs(JacsServiceData jacsServiceData) {
        LsmFileMetadataServiceDescriptor.LsmFileMetadataArgs args = new LsmFileMetadataServiceDescriptor.LsmFileMetadataArgs();
        new JCommander(args).parse(jacsServiceData.getArgsArray());
        return args;
    }

    private File getInputFile(LsmFileMetadataServiceDescriptor.LsmFileMetadataArgs args) {
        return new File(args.inputLSMFile);
    }

    private File getOutputFile(LsmFileMetadataServiceDescriptor.LsmFileMetadataArgs args) {
        try {
            File outputFile = new File(args.outputLSMMetadata);
            Files.createDirectories(outputFile.getParentFile().toPath());
            return outputFile;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

}