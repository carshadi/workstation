
package org.janelia.horta;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.stream.IntStream;

import Jama.Matrix;
import org.apache.commons.lang3.StringUtils;
import org.janelia.rendering.RawImage;

/**
 * BrainTileInfo factory.
 */
public class BrainTileInfoBuilder {
    // e.g.
    //path: /nobackup/mousebrainmicro/data/2014-04-04/Tiling
    //tiles:
    //- aabb:
    //    ori: [84934200, 17379900, 9909023]
    //    shape: [386670, 532776, 200000]
    //  path: /2014-04-14/01/01659
    //  shape:
    //    dims: [1024, 2048, 201, 2]
    //    type: u16
    //  transform: [-377.607422, 0.0, 0.0, 0.0, 85320872.0, 0.0, -260.144531, 0.0, 0.0,
    //    17912676.0, 0.0, 0.485852, 995.024902, 0.0, 9909023.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    //    0.0, 0.0, 0.0, 0.0, 1.0]

    public static BrainTileInfo fromYAMLFragment(TileLoader tileLoader, String tileBasePath, boolean leverageCompressedFiles, Map<String, Object> yamlFragment) {
        Map<String, Object> aabb = (Map<String, Object>) yamlFragment.get("aabb");
        if (aabb == null) {
            throw new IllegalArgumentException("Field missing for extracting origin and shape coordinates from " + yamlFragment);
        }
        List<Integer> ori = (List<Integer>) aabb.get("ori");
        if (ori == null) {
            throw new IllegalArgumentException("Origin coordinates are missing " + yamlFragment);
        } else if (ori.size() != 3) {
            throw new IllegalArgumentException("Expected origin coordinates to be of size 3 " + yamlFragment);
        }
        List<Integer> bbshape = (List<Integer>) aabb.get("shape");
        if (bbshape == null) {
            throw new IllegalArgumentException("Shape coordinates are missing " + yamlFragment);
        } else if (bbshape.size() != 3) {
            throw new IllegalArgumentException("Expected shape coordinates to be of size 3 " + yamlFragment);
        }
        int[] bbOriginNanometers = new int[3];
        int[] bbShapeNanometers = new int[3];
        IntStream.rangeClosed(0, 2).forEach(i -> {
            bbOriginNanometers[i] = ori.get(i);
            bbShapeNanometers[i] = bbshape.get(i);
        });
        String localPath = (String) yamlFragment.get("path");
        Map<String, Object> shape = (Map<String, Object>) yamlFragment.get("shape");
        if (shape == null) {
            throw new IllegalArgumentException("Missing shape for extracting pixel dimensions " + yamlFragment);
        }
        List<Integer> dims = (List<Integer>) shape.get("dims");
        if (dims == null) {
            throw new IllegalArgumentException("Pixel dimensions is missing " + yamlFragment);
        } else if (dims.size() != 4) {
            throw new IllegalArgumentException("Expected 4 pixel dimensions" + yamlFragment);
        }
        int[] pixelDims = new int[4];
        IntStream.rangeClosed(0, 3).forEach(i -> {
            pixelDims[i] = dims.get(i);
        });
        String intensityType = (String) shape.get("type");
        int bytesPerIntensity;
        if (StringUtils.equalsIgnoreCase("u16", intensityType)) {
            bytesPerIntensity = 2;
        } else {
            bytesPerIntensity = 1;
        }
        List<Double> td = (List<Double>) yamlFragment.get("transform");
        if (td == null) {
            throw new IllegalArgumentException("Tiles missing transform " + yamlFragment);
        }
        if (td.size() != 25) {
            throw new IllegalArgumentException("Tiles transform size should be 25 instead of " + td.size() + " in " + yamlFragment);
        }
        double[][] dd = new double[5][5];
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                dd[i][j] = td.get(5 * i + j);
            }
        }
        Matrix transform = new Matrix(dd, 5, 5);
        return new BrainTileInfo(
                tileLoader,
                tileBasePath,
                localPath,
                bbOriginNanometers,
                bbShapeNanometers,
                pixelDims,
                bytesPerIntensity,
                transform);
    }

    public static BrainTileInfo fromRawImage(TileLoader tileLoader, String acquisitionPath, RawImage rawImage) {
        double[][] dd = new double[5][5];
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                dd[i][j] = rawImage.getTransform()[5 * i + j];
            }
        }
        Matrix transform = new Matrix(dd, 5, 5);
        return new BrainTileInfo(
                tileLoader,
                acquisitionPath,
                rawImage.getRelativePath(),
                Arrays.stream(rawImage.getOriginInNanos()).mapToInt(Integer::intValue).toArray(),
                Arrays.stream(rawImage.getDimsInNanos()).mapToInt(Integer::intValue).toArray(),
                Arrays.stream(rawImage.getTileDims()).mapToInt(Integer::intValue).toArray(),
                rawImage.getBytesPerIntensity(),
                transform);
    }
}
