package org.janelia.it.FlyWorkstation.gui.framework.viewer.alignment_board;

import org.janelia.it.FlyWorkstation.gui.alignment_board_viewer.renderable.*;
import org.janelia.it.FlyWorkstation.gui.framework.session_mgr.SessionMgr;
import org.janelia.it.FlyWorkstation.gui.alignment_board_viewer.gui_elements.GpuSampler;
import org.janelia.it.FlyWorkstation.gui.viewer3d.loader.*;
import org.janelia.it.FlyWorkstation.gui.alignment_board_viewer.masking.*;
import org.janelia.it.FlyWorkstation.gui.viewer3d.resolver.CacheFileResolver;
import org.janelia.it.FlyWorkstation.gui.viewer3d.resolver.FileResolver;
import org.janelia.it.FlyWorkstation.gui.viewer3d.texture.TextureDataI;
import org.janelia.it.FlyWorkstation.gui.alignment_board_viewer.volume_builder.RenderablesChannelsBuilder;
import org.janelia.it.FlyWorkstation.gui.alignment_board_viewer.volume_builder.RenderablesMaskBuilder;
import org.janelia.it.FlyWorkstation.shared.workers.SimpleWorker;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.*;
import java.util.*;
import java.util.concurrent.*;

/**
 * Created with IntelliJ IDEA.
 * User: fosterl
 * Date: 3/29/13
 * Time: 11:16 AM
 *
 * Loads renderable-oriented data into the Alignment Board and MIP3d.
 */
public class RenderablesLoadWorker extends SimpleWorker implements VolumeLoader {

    private static final int LEAST_FULLSIZE_MEM = 1500000; // Ex: 1,565,620
    private Boolean loadFiles = true;

    private MaskChanMultiFileLoader compartmentLoader;
    private MaskChanMultiFileLoader neuronFragmentLoader;
    private RenderMappingI renderMapping;
    private FileStats fileStats;

    private RenderablesMaskBuilder maskTextureBuilder;
    private RenderablesChannelsBuilder signalTextureBuilder;
    private RenderableDataSourceI dataSource;
    private AlignmentBoardSettings alignmentBoardSettings;
    private MultiMaskTracker multiMaskTracker;

    private AlignmentBoardControllable controlCallback;
    private GpuSampler sampler;

    private FileResolver resolver;

    private Logger logger;

    public RenderablesLoadWorker(
            RenderableDataSourceI dataSource,
            RenderMappingI renderMapping,
            AlignmentBoardControllable controlCallback,
            AlignmentBoardSettings settings,
            MultiMaskTracker multiMaskTracker
    ) {
        logger = LoggerFactory.getLogger(SimpleWorker.class);
        this.dataSource = dataSource;
        this.renderMapping = renderMapping;
        this.alignmentBoardSettings = settings;
        this.controlCallback = controlCallback;
        this.multiMaskTracker = multiMaskTracker;
    }

    /**
     * This c'tor to be called when the downsample rate must be determined from graphic board.
     * @throws Exception for called methods. Particularly threading.
     */
    public RenderablesLoadWorker(
            RenderableDataSourceI dataSource,
            RenderMappingI renderMapping,
            AlignmentBoardControllable controlCallback,
            AlignmentBoardSettings settings,
            MultiMaskTracker multiMaskTracker,
            GpuSampler sampler
    ) throws Exception {
        this( dataSource, renderMapping, controlCallback, settings, multiMaskTracker );
        this.sampler = sampler;
    }

    public void setResolver( FileResolver resolver ) {
        this.resolver = resolver;
    }

    public void setLoadFilesFlag( Boolean loadFiles ) {
        this.loadFiles = loadFiles;
    }

    public FileStats getFileStats() {
        return fileStats;
    }

    public void setFileStats(FileStats fileStats) {
        this.fileStats = fileStats;
    }

    //------------------------------------------IMPLEMENTS VolumeLoader
    /**
     * Loads one renderable's data into the volume under construction.
     *
     * @param maskChanRenderableData renderable data to be applied to volume.
     * @throws Exception from called methods.
     */
    public void loadVolume( MaskChanRenderableData maskChanRenderableData ) throws Exception {
        logger.debug(
                "In load thread, STARTING load of renderable {}.",
                maskChanRenderableData.getBean().getTranslatedNum()
        );

        // Special case: the "signal" renderable will have a translated label number of zero.  It will not
        // require a file load.
        if ( maskChanRenderableData.getBean().getTranslatedNum() == 0 ) {
            return;
        }

        MaskChanStreamSource streamSource = new MaskChanStreamSource(
                maskChanRenderableData, resolver, alignmentBoardSettings.isShowChannelData()
        );

        MaskChanStreamSource.StreamSourceSanity sanity = streamSource.getSanity();
        if ( ! sanity.isSane() ) {
            logger.warn( sanity.getMessage() );
            return;
        }

        // Feed data to the acceptors.
        if ( maskChanRenderableData.isCompartment() ) {
            compartmentLoader.read(maskChanRenderableData.getBean(), streamSource);
        }
        else {
            neuronFragmentLoader.read(maskChanRenderableData.getBean(), streamSource);
        }

        logger.debug("In load thread, ENDED load of renderable {}.", maskChanRenderableData.getBean().getLabelFileNum() );
    }


    //----------------------------------------------OVERRIDE SimpleWorker
    @Override
    protected void doStuff() throws Exception {

        Collection<MaskChanRenderableData> renderableDatas = dataSource.getRenderableDatas();

        if ( loadFiles ) {
            if ( resolver == null ) {
                //resolver = new TrivialFileResolver();  // swap comments, in testing.
                resolver = new CacheFileResolver();
            }

            if ( sampler != null )
                alignmentBoardSettings = adjustDownsampleRateSetting();

            // Cut down the to-renders: use only the larger ones.
            long fragmentFilterSize = alignmentBoardSettings.getMinimumVoxelCount();
            if ( fragmentFilterSize != -1 ) {
                FragmentSizeFilter filter = new FragmentSizeFilter( fragmentFilterSize );
                renderableDatas = filter.filter( renderableDatas );
            }

            List<RenderableBean> renderableBeans = new ArrayList<RenderableBean>();
            int lastUsedMask = -1;
            for ( MaskChanRenderableData renderableData: renderableDatas ) {
                RenderableBean bean = renderableData.getBean();

                // Need to add sizing data to each renderable bean prior to sorting.
                MaskChanSingleFileLoader loader = new MaskChanSingleFileLoader( null, null, bean, null );
                if ( renderableData.getMaskPath() != null ) {
                    File infile = new File( resolver.getResolvedFilename( renderableData.getMaskPath() ) );
                    if ( infile.canRead() ) {
                        FileInputStream fis = new FileInputStream( infile );
                        long voxelCount = loader.getVoxelCount( fis );
                        bean.setVoxelCount( voxelCount );
                    }
                }
                renderableBeans.add( bean );
                if ( bean.getTranslatedNum() > lastUsedMask ) {
                    lastUsedMask = bean.getTranslatedNum();
                }
            }
            if ( lastUsedMask > -1 ) {
                multiMaskTracker.setFirstMaskNum( lastUsedMask + 1 ); // Add one to move past all allocated masks.
            }
            Collections.sort( renderableBeans, new InvertingComparator( new RBComparator() ) );

            renderMapping.setRenderables( renderableBeans );

            // Establish the means for extracting the volume mask.
            maskTextureBuilder = new RenderablesMaskBuilder( alignmentBoardSettings, renderableBeans );

            // Establish the means for extracting the signal data.
            signalTextureBuilder = new RenderablesChannelsBuilder(
                    alignmentBoardSettings, multiMaskTracker, maskTextureBuilder, renderableBeans
            );

            // Unfortunately, the wrapped thing knows what wraps it, but at least by a different interface.
            RemaskingAcceptorDecorator remaskingAcceptorDecorator = new RemaskingAcceptorDecorator(
                    maskTextureBuilder,
                    multiMaskTracker,
                    maskTextureBuilder,
                    RenderablesMaskBuilder.UNIVERSAL_MASK_BYTE_COUNT,
                    false    // NOT binary / search writeback.  That happens only for the file writeback code.
            );

            ArrayList<MaskChanDataAcceptorI> acceptors = new ArrayList<MaskChanDataAcceptorI>();
            acceptors.add(remaskingAcceptorDecorator);

            // Setup the loader to traverse all this data on demand. Only the mask-tex-builder accepts data.
            neuronFragmentLoader = new MaskChanMultiFileLoader();
            neuronFragmentLoader.setAcceptors(acceptors);
            neuronFragmentLoader.setFileStats( fileStats );

            compartmentLoader = new MaskChanMultiFileLoader();
            compartmentLoader.setAcceptors( acceptors );

            logger.info("Timing multi-thread data load for multi-mask-assbembly.");
            multiThreadedDataLoad(renderableDatas, false);
            logger.info("End timing multi-mask-assembly");

            // RE-run the scan.  This time only the signal-texture-builder will accept the data.
            acceptors.clear();
            acceptors.add( signalTextureBuilder );
            neuronFragmentLoader.setAcceptors(acceptors);
            compartmentLoader.setAcceptors( acceptors );

            logger.info("Timing multi-thread data load for signal.");
            multiThreadedDataLoad(renderableDatas, true);
            logger.info("End timing signal load");

            compartmentLoader.close();
            neuronFragmentLoader.close();

        }
        else {
            renderChange(renderableDatas);
        }

        logger.info("Ending load thread.");
    }

    @Override
    protected void hadSuccess() {
        controlCallback.loadCompletion(true, loadFiles, null);
    }

    @Override
    protected void hadError(Throwable error) {
        controlCallback.loadCompletion(false, loadFiles, error);
    }

    /**
     * Carries out all file-reading.
     *
     * @param metaDatas one thread for each of these.
     */
    private void multiThreadedDataLoad(Collection<MaskChanRenderableData> metaDatas, boolean buildTexture) {
        controlCallback.clearDisplay();

        if ( metaDatas == null  ||  metaDatas.size() == 0 ) {
            logger.info( "No renderables found for alignment board " + dataSource.getName() );
        }
        else {
            logger.debug( "In load thread, after getting bean list." );

            logger.debug("Starting multithreaded file load.");
            fileLoad(metaDatas);

            if ( buildTexture ) {
                logger.debug("Starting multithreaded texture build.");
                multiThreadedTextureBuild();
            }

        }

        controlCallback.displayReady();
    }

    /**
     * Carry out the texture building in parallel.  One thread for each texture type.
     */
    private void multiThreadedTextureBuild() {
        // Multi-threading, part two.  Here, the renderable textures are created out of inputs.
        final CyclicBarrier buildBarrier = new CyclicBarrier( 3 );
        try {
            // These two texture-build steps will proceed in parallel.
            TexBuildRunnable signalBuilderRunnable = new TexBuildRunnable( signalTextureBuilder, buildBarrier );
            TexBuildRunnable maskBuilderRunnable = new TexBuildRunnable( maskTextureBuilder, buildBarrier );

            new Thread( signalBuilderRunnable ).start();
            new Thread( maskBuilderRunnable ).start();

            buildBarrier.await();

            if ( buildBarrier.isBroken() ) {
                throw new Exception( "Tex build failed." );
            }

            TextureDataI signalTexture = signalBuilderRunnable.getTextureData();
            TextureDataI maskTexture = maskBuilderRunnable.getTextureData();

            controlCallback.loadVolume(signalTexture, maskTexture);

        } catch ( BrokenBarrierException bbe ) {
            logger.error( "Barrier await failed during texture build.", bbe );
            bbe.printStackTrace();
        } catch ( InterruptedException ie ) {
            logger.error( "Thread interrupted during texture build.", ie );
            ie.printStackTrace();
        } catch ( Exception ex ) {
            logger.error( "Exception during texture build.", ex );
            ex.printStackTrace();
        } finally {
            if ( ! buildBarrier.isBroken() ) {
                buildBarrier.reset(); // Signal to others: failed.
            }
        }
    }

    private void fileLoad( Collection<MaskChanRenderableData> metaDatas ) {
        List<MaskChanRenderableData> sortedMetaDatas = new ArrayList<MaskChanRenderableData>();
        sortedMetaDatas.addAll( metaDatas );
        Collections.sort( sortedMetaDatas, new RDComparator( false ) );
        for ( MaskChanRenderableData metaData: sortedMetaDatas ) {
            logger.debug( "Scheduling mask path {} for load as {}.", metaData.getMaskPath(), metaData.getBean().getTranslatedNum() );
            LoadRunnable runnable = new LoadRunnable( metaData, this, null );
            runnable.run();
        }

    }

    /**
     * Allows the downsample-rate setting used in populating the textures, to be adjusted based on user's
     * platform.
     *
     * @return adjusted settings.
     * @throws Exception from any called methods.
     */
    private AlignmentBoardSettings adjustDownsampleRateSetting() throws Exception {

        logger.info("Adjusting downsample rate from {}.", Thread.currentThread().getName());
        try {
            GpuSampler.GpuInfo gpuInfo = sampler.getGpuInfo();

            // Must set the down sample rate to the newly-discovered best.
            if ( gpuInfo != null ) {
                logger.info(
                        "GPU vendor {}, renderer {} version " + gpuInfo.getVersion(), gpuInfo.getVender(), gpuInfo.getRenderer()
                );

                // 1.5Gb in Kb increments
                logger.info( "ABV seeing free memory estimate of {}.", gpuInfo.getFreeTexMem() );
                logger.info( "ABV seeting highest supported version of {}.", gpuInfo.getHighestGlslVersion() );

                if ( gpuInfo.getFreeTexMem() > LEAST_FULLSIZE_MEM ) {
                    alignmentBoardSettings.setDownSampleGuess(1.0);
                }
                else if ( GpuSampler.isDeptStandardGpu( gpuInfo.getRenderer() ) ) {
                    alignmentBoardSettings.setDownSampleGuess(1.0);
                }
                else {
                    Future<Boolean> isDeptPreferred = sampler.isDepartmentStandardGraphicsMac();
                    try {
                        if ( isDeptPreferred.get() ) {
                            logger.info("User has preferred card.");
                            alignmentBoardSettings.setDownSampleGuess(1.0);
                        }
                        else {
                            alignmentBoardSettings.setDownSampleGuess(2.0);
                        }
                    } catch ( Exception ex ) {
                        logger.warn( "Ignore this message if this system is not a Mac: department-preferred grapchics detection not working on this platform." );
                    }
                }
            }
            else {
                logger.warn( "No vender data returned.  Forcing 'safe guess'." );
                alignmentBoardSettings.setDownSampleGuess(2.0);
            }

        } catch ( Exception ex ) {
            ex.printStackTrace();
            SessionMgr.getSessionMgr().handleException( ex );
        }

        //this.remove( feedbackPanel );

        //todo find some way to return this and avoid re-processing.
        //cachedDownSampleGuess = alignmentBoardSettings.getDownSampleGuess();
        return alignmentBoardSettings;
    }

    private void renderChange(Collection<MaskChanRenderableData> metaDatas) {
        if ( metaDatas.size() == 0 ) {
            logger.info("No renderables found for alignment board " + dataSource.getName());
        }
        else {
            Collection<RenderableBean> beans = new ArrayList<RenderableBean>();
            for ( MaskChanRenderableData metaData: metaDatas ) {
                beans.add( metaData.getBean() );
            }
            renderMapping.setRenderables( beans );
            controlCallback.renderModCompletion();
        }

    }

}
