package org.janelia.it.workstation.ab2.model;

import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Set;

import org.janelia.it.workstation.ab2.test.AB2SimulatedNeuronSkeletonGenerator;
import org.janelia.it.workstation.ab2.test.AB2SimulatedVolumeGenerator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class AB2SkeletonDomainObject extends AB2DomainObject {

    Logger logger= LoggerFactory.getLogger(AB2SkeletonDomainObject.class);

    ////////////////////////////////////////////////////////////////////////////////////
    /// For Skeleton
    ////////////////////////////////////////////////////////////////////////////////////

    private List<AB2NeuronSkeleton> skeletons=new ArrayList<>();
    AB2SimulatedVolumeGenerator volumeGenerator;

    public AB2SkeletonDomainObject() {}

    public void createSkeletonsAndVolume(int number) throws Exception {
        createSkeletonsAndVolume(number, new Date().getTime());
        //createSkeletons(number, 1);
    }

    public void createSkeletonsAndVolume(int number, long randomSeed) throws Exception {
        AB2SimulatedNeuronSkeletonGenerator skeletonGenerator=new AB2SimulatedNeuronSkeletonGenerator(randomSeed);
        for (int i=0;i<number;i++) {
            logger.info("Generating skeleton "+i+" of "+number);
            AB2NeuronSkeleton skeleton = skeletonGenerator.generateSkeleton();
            skeletons.add(skeleton);
        }
        volumeGenerator=new AB2SimulatedVolumeGenerator(512, 512, 512); // 800 X 3, close to limit for byte array length

        for (int i=0;i<skeletons.size();i++) {
            logger.info("Skeleton "+i);
            volumeGenerator.addSkeleton(skeletons.get(i));
        }

//        logger.info("Added all skeletons to Simulated Volume");
//
//        logger.info("Starting dilation 1");
//
//        volumeGenerator.performDilation(3.0f, 0.0f);
//
//        logger.info("Dilation finished 1");
//
//        logger.info("Starting dilation 2");
//
//        volumeGenerator.performDilation(3.0f, 0.0f);
//
//        logger.info("Dilation finished 2");

    }

    public List<AB2NeuronSkeleton> getSkeletons() { return skeletons; }

    public AB2SimulatedVolumeGenerator getVolumeGenerator() {
        return volumeGenerator;
    }
}
