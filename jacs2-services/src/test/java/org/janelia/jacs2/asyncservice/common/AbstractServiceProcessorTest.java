package org.janelia.jacs2.asyncservice.common;

import com.google.common.collect.ImmutableList;
import com.offbynull.coroutines.user.CoroutineRunner;
import org.janelia.jacs2.asyncservice.JacsServiceEngine;
import org.janelia.jacs2.model.jacsservice.JacsServiceData;
import org.janelia.jacs2.dataservice.persistence.JacsServiceDataPersistence;
import org.janelia.jacs2.model.jacsservice.JacsServiceState;
import org.janelia.jacs2.model.jacsservice.ServiceMetaData;
import org.junit.Before;
import org.junit.Test;
import org.slf4j.Logger;

import java.util.List;
import java.util.function.Consumer;

import static org.hamcrest.CoreMatchers.equalTo;
import static org.junit.Assert.assertThat;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

public class AbstractServiceProcessorTest {

    private static final String TEST_WORKING_DIR = "testDir";
    private static final Number TEST_ID = 1L;

    static class TestSuccessfulProcessor extends AbstractServiceProcessor<Void> {

        private long timeout = -1;

        public TestSuccessfulProcessor(JacsServiceEngine jacsServiceEngine,
                                       ServiceComputationFactory computationFactory,
                                       JacsServiceDataPersistence jacsServiceDataPersistence,
                                       String defaultWorkingDir,
                                       Logger logger) {
            super(jacsServiceEngine, computationFactory, jacsServiceDataPersistence, defaultWorkingDir, logger);
        }

        @Override
        public ServiceMetaData getMetadata() {
            return new ServiceMetaData();
        }

        @Override
        public Void getResult(JacsServiceData jacsServiceData) {
            return null;
        }

        @Override
        public void setResult(Void result, JacsServiceData jacsServiceData) {
        }

        @Override
        protected ServiceComputation<JacsServiceData> preProcessData(JacsServiceData jacsServiceData) {
            if (timeout > 0) {
                try {
                    Thread.sleep(timeout);
                } catch (InterruptedException e) {
                }
            }
            return createComputation(jacsServiceData);
        }

        @Override
        protected ServiceComputation<JacsServiceData> processData(JacsServiceData jacsServiceData) {
            return createComputation(jacsServiceData);
        }

        @Override
        protected List<JacsServiceData> submitAllDependencies(JacsServiceData jacsServiceData) {
            return ImmutableList.of();
        }

        @Override
        protected boolean isResultAvailable(JacsServiceData jacsServiceData) {
            return true;
        }

        @Override
        protected Void retrieveResult(JacsServiceData jacsServiceData) {
            return null;
        }

        public void setTimeout(long timeout) {
            this.timeout = timeout;
        }
    }

    static class TestFailedProcessor extends AbstractServiceProcessor<Void> {

        public TestFailedProcessor(JacsServiceEngine jacsServiceEngine,
                                   ServiceComputationFactory computationFactory,
                                   JacsServiceDataPersistence jacsServiceDataPersistence,
                                   String defaultWorkingDir,
                                   Logger logger) {
            super(jacsServiceEngine, computationFactory, jacsServiceDataPersistence, defaultWorkingDir, logger);
        }

        @Override
        public ServiceMetaData getMetadata() {
            return new ServiceMetaData();
        }

        @Override
        public Void getResult(JacsServiceData jacsServiceData) {
            return null;
        }

        @Override
        public void setResult(Void result, JacsServiceData jacsServiceData) {
        }

        @Override
        protected ServiceComputation<JacsServiceData> processData(JacsServiceData jacsServiceData) {
            return createFailure(jacsServiceData, new ComputationException(jacsServiceData, "test"));
        }

        @Override
        protected List<JacsServiceData> submitAllDependencies(JacsServiceData jacsServiceData) {
            return ImmutableList.of();
        }

        @Override
        protected boolean isResultAvailable(JacsServiceData jacsServiceData) {
            return false;
        }

        @Override
        protected Void retrieveResult(JacsServiceData jacsServiceData) {
            return null;
        }
    }

    private JacsServiceDataPersistence jacsServiceDataPersistence;
    private JacsServiceEngine jacsServiceEngine;
    private ServiceComputationFactory serviceComputationFactory;
    private Logger logger;

    private TestSuccessfulProcessor testSuccessfullProcessor;
    private TestFailedProcessor testFailedProcessor;

    private JacsServiceData testJacsServiceData;

    @Before
    public void setUp() {
        ServiceComputationQueue serviceComputationQueue = mock(ServiceComputationQueue.class);
        doAnswer((invocation -> {
            ServiceComputationTask task = invocation.getArgument(0);
            if (task != null) {
                ServiceComputationQueue.runTask(task);
            }
            return null;
        })).when(serviceComputationQueue).submit(any(ServiceComputationTask.class));
        serviceComputationFactory = new ServiceComputationFactory(serviceComputationQueue);

        testJacsServiceData = new JacsServiceData();
        testJacsServiceData.setId(TEST_ID);
        jacsServiceEngine = mock(JacsServiceEngine.class);
        jacsServiceDataPersistence = mock(JacsServiceDataPersistence.class);

        logger = mock(Logger.class);
        testSuccessfullProcessor = new TestSuccessfulProcessor(
                jacsServiceEngine,
                serviceComputationFactory,
                jacsServiceDataPersistence,
                TEST_WORKING_DIR,
                logger);
        testFailedProcessor = new TestFailedProcessor(
            jacsServiceEngine,
                serviceComputationFactory,
                jacsServiceDataPersistence,
                TEST_WORKING_DIR,
                logger);
    }

    @Test
    public void successfulProcessing() {
        Consumer successful = mock(Consumer.class);
        Consumer failure = mock(Consumer.class);

        JacsServiceData testJacsServiceDataDependency = new JacsServiceData();
        testJacsServiceDataDependency.setId(TEST_ID.longValue() + 1);

        testJacsServiceData.addServiceDependency(testJacsServiceDataDependency);
        when(jacsServiceDataPersistence.findById(TEST_ID)).thenReturn(testJacsServiceData);
        when(jacsServiceDataPersistence.findServiceHierarchy(TEST_ID))
                .thenAnswer(invocation -> {
                    testJacsServiceDataDependency.setState(JacsServiceState.SUCCESSFUL);
                    return testJacsServiceData;
                });

        testSuccessfullProcessor.process(testJacsServiceData)
                .whenComplete((r, e) -> {
                   if (e == null) {
                       successful.accept(r);
                   } else {
                       failure.accept(e);
                   }
                });
        verify(failure, never()).accept(any());
        verify(successful).accept(any());
    }

    @Test
    public void processingFailureDueToDependencyFailure() {
        Consumer successful = mock(Consumer.class);
        Consumer failure = mock(Consumer.class);

        JacsServiceData testJacsServiceDataDependency = new JacsServiceData();
        testJacsServiceDataDependency.setId(TEST_ID.longValue() + 1);

        testJacsServiceData.addServiceDependency(testJacsServiceDataDependency);
        when(jacsServiceDataPersistence.findServiceHierarchy(TEST_ID))
                .thenAnswer(invocation -> {
                    testJacsServiceDataDependency.setState(JacsServiceState.CANCELED);
                    return testJacsServiceData;
                });

        testSuccessfullProcessor.process(testJacsServiceData)
                .whenComplete((r, e) -> {
                    if (e == null) {
                        successful.accept(r);
                    } else {
                        failure.accept(e);
                    }
                });
        verify(failure).accept(any());
        verify(successful, never()).accept(any());
    }

    @Test
    public void processingSuspended() {
        Consumer successful = mock(Consumer.class);
        Consumer failure = mock(Consumer.class);

        JacsServiceData testJacsServiceDataDependency = new JacsServiceData();
        testJacsServiceDataDependency.setId(TEST_ID.longValue() + 1);
        testJacsServiceDataDependency.setState(JacsServiceState.RUNNING);

        testJacsServiceData.addServiceDependency(testJacsServiceDataDependency);

        when(jacsServiceDataPersistence.findById(TEST_ID)).thenReturn(testJacsServiceData);
        when(jacsServiceDataPersistence.findServiceHierarchy(TEST_ID)).thenReturn(testJacsServiceData);

        testSuccessfullProcessor.process(testJacsServiceData)
                .whenComplete((r, e) -> {
                    if (e == null) {
                        successful.accept(r);
                    } else {
                        failure.accept(e);
                    }
                });
        verify(successful).accept(any());
        verify(failure, never()).accept(any());
        assertThat(testJacsServiceData.getState(), equalTo(JacsServiceState.SUSPENDED));
    }

    @Test
    public void failedProcessing() throws ComputationException {
        Consumer successful = mock(Consumer.class);
        Consumer failure = mock(Consumer.class);

        testFailedProcessor.process(testJacsServiceData)
                .whenComplete((r, e) -> {
                    if (e == null) {
                        successful.accept(r);
                    } else {
                        failure.accept(e);
                    }
                });
        verify(successful, never()).accept(any());
        verify(failure).accept(any());
    }

    @Test
    public void successfulWaitForDependencies() {
        Consumer successful = mock(Consumer.class);
        Consumer failure = mock(Consumer.class);

        JacsServiceData testJacsServiceDataDependency = new JacsServiceData();
        testJacsServiceDataDependency.setId(TEST_ID.longValue() + 1);
        testJacsServiceDataDependency.setState(JacsServiceState.RUNNING);

        testJacsServiceData.addServiceDependency(testJacsServiceDataDependency);

        when(jacsServiceDataPersistence.findById(TEST_ID)).thenReturn(testJacsServiceData);
        when(jacsServiceDataPersistence.findServiceHierarchy(TEST_ID))
                .thenAnswer(invocation -> {
                    testJacsServiceDataDependency.setState(JacsServiceState.SUCCESSFUL);
                    return testJacsServiceData;
                });

        when(jacsServiceEngine.getServiceProcessor(testJacsServiceData)).thenReturn((ServiceProcessor) testSuccessfullProcessor);

        testSuccessfullProcessor.waitForDependencies(testJacsServiceData)
                .whenComplete((r, e) -> {
                    if (e == null) {
                        successful.accept(r);
                    } else {
                        failure.accept(e);
                    }
                });
        verify(failure, never()).accept(any());
        verify(successful).accept(any());
    }

    @Test
    public void failedDependencies() {
        Consumer successful = mock(Consumer.class);
        Consumer failure = mock(Consumer.class);

        JacsServiceData testJacsServiceDataDependency = new JacsServiceData();
        testJacsServiceDataDependency.setId(TEST_ID.longValue() + 1);
        testJacsServiceDataDependency.setState(JacsServiceState.RUNNING);

        testJacsServiceData.addServiceDependency(testJacsServiceDataDependency);

        when(jacsServiceDataPersistence.findById(TEST_ID))
                .thenAnswer(invocation -> testJacsServiceData)
                .thenAnswer(invocation -> {
                    testJacsServiceDataDependency.setState(JacsServiceState.ERROR);
                    return testJacsServiceData;
                });

        when(jacsServiceEngine.getServiceProcessor(testJacsServiceData)).thenReturn((ServiceProcessor) testSuccessfullProcessor);

        testSuccessfullProcessor.waitForDependencies(testJacsServiceData)
                .whenComplete((r, e) -> {
                    if (e == null) {
                        successful.accept(r);
                    } else {
                        failure.accept(e);
                    }
                });
        verify(successful, never()).accept(any());
        verify(failure).accept(any());
    }

    @Test
    public void waitForDependenciesAndTimeout() {
        Consumer successful = mock(Consumer.class);
        Consumer failure = mock(Consumer.class);

        JacsServiceData testJacsServiceDataDependency = new JacsServiceData();
        testJacsServiceDataDependency.setId(TEST_ID.longValue() + 1);
        testJacsServiceDataDependency.setState(JacsServiceState.RUNNING);

        testJacsServiceData.setServiceTimeout(10L);
        testJacsServiceData.addServiceDependency(testJacsServiceDataDependency);

        when(jacsServiceDataPersistence.findServiceHierarchy(TEST_ID)).thenReturn(testJacsServiceData);
        when(jacsServiceEngine.getServiceProcessor(testJacsServiceData)).thenReturn((ServiceProcessor) testSuccessfullProcessor);

        testSuccessfullProcessor.setTimeout(100L);
        testSuccessfullProcessor.waitForDependencies(testJacsServiceData)
                .whenComplete((r, e) -> {
                    if (e == null) {
                        successful.accept(r);
                    } else {
                        failure.accept(e);
                    }
                });
        verify(successful, never()).accept(any());
        verify(failure).accept(any());
        assertThat(testJacsServiceData.getState(), equalTo(JacsServiceState.TIMEOUT));
    }

}