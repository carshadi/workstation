package org.janelia.workstation.tracing;

import java.nio.ByteBuffer;
import java.nio.ShortBuffer;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.Vector;
import org.janelia.workstation.gui.large_volume_viewer.Subvolume;
import org.janelia.workstation.octree.ZoomLevel;
import org.janelia.workstation.octree.ZoomedVoxelIndex;

import com.google.common.collect.Lists;
import org.janelia.workstation.raster.VoxelIndex;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Maybe implement my own version of AStar.
 * http://en.wikipedia.org/wiki/A*_search_algorithm
 * 
 * @author brunsc
 *
 * TODO - store log(probability), not probability
 */
public class AStar {
    enum DistanceMetric {
        EUCLIDEAN,
        MANHATTAN,
    }
    private DistanceMetric distanceMetric = DistanceMetric.EUCLIDEAN;
    
    // How many neighbors to examine for each voxel?
    // TODO - not implemented yet
    enum NeighborClass {
        SIX_CONNECTED,
        // EIGHTEEN_CONNECTED, // I don't have all day here...
        TWENTYSIX_CONNECTED,
    }
    private NeighborClass neighborClass = NeighborClass.TWENTYSIX_CONNECTED;
    
    private boolean debug = false;
    // Cached values
    // stepCostLowerBound has a dramatic effect on performance 9-25-2013
    // Numbers larger than <some small amount> take more time and cause more nodes to be explored.
    // Non-zero values prevent meandering path.
    private final double stepCostLowerBound = 1e-60;
    
    private double minStepCost = Double.NaN; // will be set from volume statistics
    private Subvolume volume;
    private Map<Integer, Double> pathCostForIntensity = new HashMap<Integer, Double>();
    private double meanIntensity = Double.NaN;
    private double stdDevIntensity = Double.NaN;
    // In case of anisotropic voxel size
    private double voxelSizeX = 1.0;
    private double voxelSizeY = 1.0;
    private double voxelSizeZ = 1.0;
    Map<VoxelIndex, Node> allNodes = new HashMap<VoxelIndex, Node>();

    private static final Logger log = LoggerFactory.getLogger(AStar.class);

    public AStar(Subvolume volume) {
        this.volume = volume;
        computeIntensityStats();
    }
    
    public void setVoxelSizes(double[] sizes) {
        setVoxelSizes(sizes[0], sizes[1], sizes[2]);
    }
    public void setVoxelSizes(double xSize, double ySize, double zSize) {
        voxelSizeX = xSize;
        voxelSizeY = ySize;
        voxelSizeZ = zSize;
    }
    
//    public List<ZoomedVoxelIndex> trace(
//            ZoomedVoxelIndex start0,
//            ZoomedVoxelIndex goal0,
//            double timout)
//    {
//        Node start = getNode(new VoxelIndex(
//                start0.getX() - volume.getOrigin().getX(), 
//                start0.getY() - volume.getOrigin().getY(), 
//                start0.getZ() - volume.getOrigin().getZ()));
//        Node goal = getNode(new VoxelIndex(
//                goal0.getX() - volume.getOrigin().getX(), 
//                goal0.getY() - volume.getOrigin().getY(), 
//                goal0.getZ() - volume.getOrigin().getZ()));
//        // 
//        // The set of nodes already evaluated
//        Set<Node> closedSet = new HashSet<Node>();
//        // The set of tentative nodes to be evaluated, initially containing the start node.
//        Set<Node> openSet = new HashSet<Node>();
//        openSet.add(start);
//        // The map of navigated nodes
//        // Map<VoxelIndex, VoxelIndex> cameFrom = new HashMap<VoxelIndex, VoxelIndex>();
//        // Cost from start along best known path
//        // Map<VoxelIndex, Double> gScore = new HashMap<VoxelIndex, Double>();
//        // Map<VoxelIndex, Double> fScore = new HashMap<VoxelIndex, Double>();
//        //
//        start.gScore = 0.0;
//        start.fScore = start.gScore + heuristicCostEstimate(start.index, goal.index);
//
//        long startTime = System.currentTimeMillis();
//        long checkedVoxelCount = 0;
//        while (openSet.size() > 0) {
//            // Get node with lowest fScore in openSet
//            // TODO - perhaps a sorted list could be maintained, to avoid 
//            // exhaustive search each time.
//            Node current = null;
//            for (Node n : openSet) {
//                if (current == null) {
//                    current = n;
//                    continue;
//                }
//                if (n.fScore < current.fScore)
//                    current = n;
//            }
//            if (current.equals(goal)) {
//                if (debug) {
//                    System.out.println("Examined "+checkedVoxelCount+" voxels");
//                }
//                return reconstructPath(start, goal, start0.getZoomLevel());
//            }
//            // Remove current from openSet
//            openSet.remove(current);
//            closedSet.add(current);
//            checkedVoxelCount += 1;
//            if (debug && checkedVoxelCount % 10000 == 0)
//                System.out.println("Examined "+checkedVoxelCount+" voxels");
//            // check timeout; quick tests suggest about 5k/sec, so this ought to
//            //  be more than sensitive enough
//            if (checkedVoxelCount % 1000 == 0) {
//                if (System.currentTimeMillis() - startTime > timout * 1000) {
//                    if (debug) {
//                        System.out.println("A-star tracing timed out");
//                        System.out.println("Examined "+checkedVoxelCount+" voxels");
//                    }
//                    log.warn("A-star tracing timed out, " + checkedVoxelCount + " voxels examined");
//                    return null;
//                }
//            }
//            for (VoxelIndex neighborIndex : getNeighbors(current.index))
//            {
//                Node neighbor = getNode(neighborIndex);
//                double tentativeGScore = current.gScore
//                        + distanceBetween(current.index, neighbor.index);
//                if ( closedSet.contains(neighbor)
//                       && (tentativeGScore >= neighbor.gScore ) ) {
//                    continue;
//                }
//                if ( (! openSet.contains(neighbor)) 
//                       || (tentativeGScore < neighbor.gScore) )
//                {
//                    neighbor.cameFrom = current;
//                    neighbor.gScore = tentativeGScore;
//                    neighbor.fScore = neighbor.gScore + heuristicCostEstimate(neighbor.index, goal.index);
//                    openSet.add(neighbor);
//                }
//            }
//        }
//        return null;
//    }
    
	public List<ZoomedVoxelIndex> trace(
			ZoomedVoxelIndex start0, 
			ZoomedVoxelIndex goal0, 
			double timout) {
		// Bidirectional variant
		Node start = getNode(new VoxelIndex(
				start0.getX() - volume.getOrigin().getX(),
				start0.getY() - volume.getOrigin().getY(), 
				start0.getZ() - volume.getOrigin().getZ()));
		Node goal = getNode(new VoxelIndex(
				goal0.getX() - volume.getOrigin().getX(),
				goal0.getY() - volume.getOrigin().getY(), 
				goal0.getZ() - volume.getOrigin().getZ()));

		// The set of nodes already evaluated
		Set<Node> closedSet = new HashSet<Node>();
		// The sets of tentative nodes to be evaluated, initially containing the start and goal nodes.
		Comparator<Node> nodeComparator = new NodeComparator();
		PriorityQueue<Node> openSetA = new PriorityQueue<Node>(10, nodeComparator);
		PriorityQueue<Node> openSetB = new PriorityQueue<Node>(10, nodeComparator);
		openSetA.add(start);
		openSetB.add(goal);
		
		Map<Node, Node> parentsA = new HashMap<Node, Node>();
		Map<Node, Node> parentsB = new HashMap<Node, Node>();
		
		Map<Node, Double> distanceA = new HashMap<Node, Double>();
		Map<Node, Double> distanceB = new HashMap<Node, Double>();

		double bestPathLength = Double.POSITIVE_INFINITY;
		double fA = heuristicCostEstimate(start.index, goal.index);
		double fB = heuristicCostEstimate(goal.index, start.index);
		
		start.fScore = fA;
		goal.fScore = fB;
		
		Node touchNode = null;
		
		parentsA.put(start, null);
		parentsB.put(goal, null);
		
		distanceA.put(start, 0d);
		distanceB.put(goal, 0d);

		while (!openSetA.isEmpty() && !openSetB.isEmpty()) {
			if (openSetA.size() < openSetB.size()) {
				Node current = openSetA.poll();
				closedSet.add(current);

				if (distanceA.get(current) + heuristicCostEstimate(current.index, goal.index)
						- heuristicCostEstimate(goal.index, goal.index) >= bestPathLength
						|| distanceA.get(current) + fB
								- heuristicCostEstimate(current.index, start.index) >= bestPathLength) {
					// reject current node
					continue;
				} else {
					// stabilize current node
					for (VoxelIndex childIndex : getNeighbors(current.index)) {
						Node child = getNode(childIndex);
						if (closedSet.contains(child)) {
							continue;
						}

						double tentativeScore = distanceA.get(current) + distanceBetween(current.index, child.index);

						if (!distanceA.containsKey(child)) {
							distanceA.put(child, tentativeScore);
							parentsA.put(child, current);
							child.fScore = tentativeScore + heuristicCostEstimate(child.index, goal.index);
							openSetA.add(child);

							if (distanceB.containsKey(child)) {
								double pathLength = tentativeScore + distanceB.get(child);
								if (bestPathLength > pathLength) {
									bestPathLength = pathLength;
									touchNode = child;
								}
							}

						} else if (distanceA.get(child) > tentativeScore) {
							distanceA.put(child, tentativeScore);
							parentsA.put(child, current);
							// decrease priority
							openSetA.remove(child);
							openSetA.add(child);

							if (distanceB.containsKey(child)) {
								double pathLength = tentativeScore + distanceB.get(child);
								if (bestPathLength > pathLength) {
									bestPathLength = pathLength;
									touchNode = child;
								}
							}
						}
					}
				}
				if (!openSetA.isEmpty()) {
					Node node = openSetA.peek();
					fA = distanceA.get(node) + heuristicCostEstimate(node.index, goal.index);
				}
			} else {
				Node current = openSetB.poll();
				closedSet.add(current);

				if (distanceB.get(current) + heuristicCostEstimate(current.index, start.index)
						- heuristicCostEstimate(start.index, start.index) >= bestPathLength
						|| distanceB.get(current) + fA
								- heuristicCostEstimate(current.index, goal.index) >= bestPathLength) {
					// reject current node
					continue;
				} else {
					// stabilize current node
					for (VoxelIndex parentIndex : getNeighbors(current.index)) {
						Node parent = getNode(parentIndex);
						if (closedSet.contains(parent)) {
							continue;
						}
						
						double tentativeScore = distanceB.get(current) + distanceBetween(parent.index, current.index);
						
						if (!distanceB.containsKey(parent)) {
							distanceB.put(parent, tentativeScore);
							parentsB.put(parent, current);
							parent.fScore = tentativeScore + heuristicCostEstimate(parent.index, start.index);
							openSetB.add(parent);

							if (distanceA.containsKey(parent)) {
								double pathLength = tentativeScore + distanceA.get(parent);
								if (bestPathLength > pathLength) {
									bestPathLength = pathLength;
									touchNode = parent;
								}
							}

						} else if (distanceB.get(parent) > tentativeScore) {
							distanceB.put(parent, tentativeScore);
							parentsB.put(parent, current);
							parent.fScore = tentativeScore + heuristicCostEstimate(parent.index, start.index);
							// decrease priority
							openSetB.remove(parent);
							openSetB.add(parent);

							if (distanceA.containsKey(parent)) {
								double pathLength = tentativeScore + distanceA.get(parent);
								if (bestPathLength > pathLength) {
									bestPathLength = pathLength;
									touchNode = parent;
								}
							}
						}
					}
				}
			}
		}
		if (touchNode == null) {
			return null;
		}

		return reconstructPath(touchNode, parentsA, parentsB, start0.getZoomLevel());

	}
    
    private double distanceBetween(VoxelIndex current, VoxelIndex neighbor) {
        // Set distance to cost of second node.
        int intensity = volume.getIntensityLocal(neighbor, 0);
        double pathScore = getPathStepCostForIntensity(intensity);
        // Use Manhattan distance, and prohibit diagonal moves (for performance)
        double dx = (current.getX() - neighbor.getX()) * voxelSizeX;
        double dy = (current.getY() - neighbor.getY()) * voxelSizeY;
        double dz = (current.getZ() - neighbor.getZ()) * voxelSizeZ;
        double distance = 0;
        if (distanceMetric == DistanceMetric.MANHATTAN) {
            distance += Math.abs(dx);
            distance += Math.abs(dy);
            distance += Math.abs(dz);
        } else if (distanceMetric == DistanceMetric.EUCLIDEAN) {
            distance = Math.sqrt(dx*dx + dy*dy + dz*dz);
        }
        return pathScore * distance;
    }
    
    private List<VoxelIndex> getNeighbors(VoxelIndex center) {
        // For performance, don't step to diagonals
        // Thus, up to six neighbors in 3D
        List<VoxelIndex> result = new Vector<VoxelIndex>();
        //
        if (neighborClass == NeighborClass.SIX_CONNECTED) {
            if (center.getX() > 0)
                result.add(new VoxelIndex(center.getX()-1, center.getY(), center.getZ()));
            if (center.getY() > 0)
                result.add(new VoxelIndex(center.getX(), center.getY()-1, center.getZ()));
            if (center.getZ() > 0)
                result.add(new VoxelIndex(center.getX(), center.getY(), center.getZ()-1));
            //
            if (center.getX() < volume.getExtent().getX() - 1)
                result.add(new VoxelIndex(center.getX()+1, center.getY(), center.getZ()));
            if (center.getX() < volume.getExtent().getY() - 1)
                result.add(new VoxelIndex(center.getX(), center.getY()+1, center.getZ()));
            if (center.getX() < volume.getExtent().getZ() - 1)
                result.add(new VoxelIndex(center.getX(), center.getY(), center.getZ()+1));
        }
        else if (neighborClass == NeighborClass.TWENTYSIX_CONNECTED) {            
            for (int dx = -1; dx <= 1; ++dx) {
                int x = center.getX() + dx;
                if (x < 0) continue;
                if (x >= volume.getExtent().getX() - 1) continue;
                for (int dy = -1; dy <= 1; ++dy) {
                    int y = center.getY() + dy;
                    if (y < 0) continue;
                    if (y >= volume.getExtent().getY() - 1) continue;
                    for (int dz = -1; dz <= 1; ++dz) {
                        if ((dx == 0) && (dy == 0) && (dz == 0)) 
                            continue; // self is not a neighbor
                        int z = center.getZ() + dz;
                        if (z < 0) continue;
                        if (z >= volume.getExtent().getZ() - 1) continue;
                        result.add(new VoxelIndex(x, y, z));
                    }
                }
            }
        }
        //
        return result;
    }
    
    private Node getNode(VoxelIndex index) {
        if (! allNodes.containsKey(index))
            allNodes.put(index, new Node(index));
        return allNodes.get(index);
    }
    
//    private List<ZoomedVoxelIndex> reconstructPath(
//            Node start,
//            Node goal,
//            ZoomLevel zoomLevel) 
//    {
//        List<ZoomedVoxelIndex> result = new Vector<ZoomedVoxelIndex>();
//        Node p = goal;
//        while (! p.equals(start)) {
//            result.add(new ZoomedVoxelIndex(zoomLevel, 
//                    p.index.getX() + volume.getOrigin().getX(), 
//                    p.index.getY() + volume.getOrigin().getY(), 
//                    p.index.getZ() + volume.getOrigin().getZ()));
//            p = p.cameFrom;
//        }
//        result.add(new ZoomedVoxelIndex(zoomLevel, 
//                start.index.getX() + volume.getOrigin().getX(), 
//                start.index.getY() + volume.getOrigin().getY(), 
//                start.index.getZ() + volume.getOrigin().getZ()));
//        return Lists.reverse(result);
//    }
    
    private List<ZoomedVoxelIndex> reconstructPath(
    		Node touchNode,
    		Map<Node, Node> parentsA,
    		Map<Node, Node> parentsB,
    		ZoomLevel zoomLevel)
    {
    	List<ZoomedVoxelIndex> forwardPath = new Vector<ZoomedVoxelIndex>();
    	Node p = parentsA.get(touchNode);
    	while (p != null) {
    		forwardPath.add(new ZoomedVoxelIndex(zoomLevel,
    				p.index.getX() + volume.getOrigin().getX(),
    				p.index.getY() + volume.getOrigin().getY(),
    				p.index.getZ() + volume.getOrigin().getZ()));
    		p = parentsA.get(p);
    	}
    	List<ZoomedVoxelIndex> reversedForwardPath = Lists.reverse(forwardPath);
    	
    	List<ZoomedVoxelIndex> backwardsPath = new Vector<ZoomedVoxelIndex>();
    	p = parentsB.get(touchNode);
    	while (p != null) {
    		backwardsPath.add(new ZoomedVoxelIndex(zoomLevel,
    				p.index.getX() + volume.getOrigin().getX(),
    				p.index.getY() + volume.getOrigin().getY(),
    				p.index.getZ() + volume.getOrigin().getZ()));
    		p = parentsB.get(p);
    	}
    	
    	List<ZoomedVoxelIndex> finalPath = new Vector<ZoomedVoxelIndex>();
    	for (ZoomedVoxelIndex n : reversedForwardPath) {
    		finalPath.add(n);
    	}
    	finalPath.add(new ZoomedVoxelIndex(zoomLevel,
    			touchNode.index.getX() + volume.getOrigin().getX(),
    			touchNode.index.getY() + volume.getOrigin().getY(),
    			touchNode.index.getZ() + volume.getOrigin().getZ()));
    	for (ZoomedVoxelIndex m : backwardsPath) {
    		finalPath.add(m);
    	}
    	return finalPath;
    }
    

    // Compute mean, standard deviation, and minimum path score
    void computeIntensityStats() {
        double sumIntensity = 0;
        long intensityCount = 0;
        int maxIntensity = Integer.MIN_VALUE;
        ByteBuffer intensityBytes = volume.getByteBuffer();
        // Mean and min path
        if (volume.getBytesPerIntensity() == 2) {
            // two bytes per value ushort
            ShortBuffer shorts = intensityBytes.asShortBuffer();
            shorts.rewind();
            while (shorts.hasRemaining()) {
                int intensity = shorts.get() & 0xffff;
                maxIntensity = Math.max(intensity, maxIntensity);
                sumIntensity += intensity;
                intensityCount += 1;
            }
        }
        else { // one byte per value ubyte
            intensityBytes.rewind();
            while (intensityBytes.hasRemaining()) {
                int intensity = intensityBytes.get() & 0xff;
                maxIntensity = Math.max(intensity, maxIntensity);
                sumIntensity += intensity;
                intensityCount += 1;
            }
        }
        meanIntensity = 0.0;
        if (intensityCount > 0)
            meanIntensity = sumIntensity / (double)intensityCount;
        // Standard deviation
        double delta = 0;
        if (volume.getBytesPerIntensity() == 2) {
            // two bytes per value ushort
            ShortBuffer shorts = intensityBytes.asShortBuffer();
            shorts.rewind();
            while (shorts.hasRemaining()) {
                int intensity = shorts.get() & 0xffff;
                double di = meanIntensity - intensity;
                delta += di * di;
            }
        }
        else { // one byte per value ubyte
            intensityBytes.rewind();
            while (intensityBytes.hasRemaining()) {
                int intensity = intensityBytes.get() & 0xff;
                double di = meanIntensity - intensity;
                delta += di * di;
            }
        }
        stdDevIntensity = 1.0;
        if (intensityCount > 0) 
            stdDevIntensity = Math.sqrt(delta/(double)intensityCount);
        // minStepCost must be computed AFTER mean/stddev
        minStepCost = getPathStepCostForIntensity(maxIntensity) 
                + stepCostLowerBound
                ;
    }
    
    // fractional error in math formula less than 1.2 * 10 ^ -7.
    // although subject to catastrophic cancellation when z in very close to 0
    // from Chebyshev fitting formula for erf(z) from Numerical Recipes, 6.2
    // CMB - return 1-erf for better numerical precision at high Z
    public static double oneMinusErf(double z) {
        double t = 1.0 / (1.0 + 0.5 * Math.abs(z));

        // use Horner's method
        double result = t * Math.exp( -z*z   -   1.26551223 +
                                            t * ( 1.00002368 +
                                            t * ( 0.37409196 + 
                                            t * ( 0.09678418 + 
                                            t * (-0.18628806 + 
                                            t * ( 0.27886807 + 
                                            t * (-1.13520398 + 
                                            t * ( 1.48851587 + 
                                            t * (-0.82215223 + 
                                            t * ( 0.17087277))))))))));
        if (z < 0) 
            result = 2.0 - result;
        return  result;
    }

    // Let path step cost be the probability that this intensity could 
    // occur by chance, given the intensity statistics.
    private double getPathStepCostForIntensity(int intensity) {
        double result;
        if (pathCostForIntensity.containsKey(intensity))
            result = pathCostForIntensity.get(intensity);
        else {
            double zScore = (intensity - meanIntensity) / stdDevIntensity;
            // Reduce Z-score by a factor, so we can numerically distinguish more very bright values
            final double zFudge = 0.80;
            result = oneMinusErf(zFudge*zScore);
            // Store computed value for future use
            pathCostForIntensity.put(intensity, result);
        }
        return result;
    }

    // Must not overestimate actual cost of path to goal
    double heuristicCostEstimate(VoxelIndex v1, VoxelIndex v2) {
        double dx = (v1.getX() - v2.getX()) * voxelSizeX;
        double dy = (v1.getY() - v2.getY()) * voxelSizeY;
        double dz = (v1.getZ() - v2.getZ()) * voxelSizeZ;
        double distance = 0;
        if (distanceMetric == DistanceMetric.MANHATTAN) {
            // Use Manhattan distance, and prohibit diagonal moves (for performance)
            distance += Math.abs(dx);
            distance += Math.abs(dy);
            distance += Math.abs(dz);
        } else if (distanceMetric == DistanceMetric.EUCLIDEAN) {
            // TODO - cache the 6? possible distances
            distance = Math.sqrt(dx*dx + dy*dy + dz*dz);
        }
        return distance * minStepCost;
    }
    
    static class Node {
        Node(VoxelIndex index) {
            this.index = index;
        }

        @Override
        public int hashCode() {
            return index.hashCode();
        }
        
        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            Node other = (Node) obj;
            return index.equals(other.index);
        }

        double fScore = Double.NaN;
        double gScore = Double.NaN;
        Node cameFrom = null;
        VoxelIndex index;
    }
    
	static class NodeComparator implements Comparator<Node> {
		@Override
		public int compare(Node n1, Node n2) {
			if (n1.fScore < n2.fScore) {
				return -1;
			}
			if (n1.fScore > n2.fScore) {
				return 1;
			}
			return 0;
		}
	}
}
