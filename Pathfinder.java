package com.example.PiggyWalker.webwalker.util;

import com.example.PiggyWalker.PathStep;
import com.example.PiggyWalker.webwalker.reader.CollisionMap;
import com.example.PiggyWalker.webwalker.reader.Flags;
import com.example.PiggyWalker.webwalker.transportations.Direction;
import com.example.PiggyWalker.webwalker.transportations.Transportation;
import net.runelite.api.coords.WorldPoint;

import java.time.Duration;
import java.time.Instant;
import java.util.*;

/**
 * An A* pathfinding implementation with support for shortcuts (transportations).
 * 
 * Key improvements:
 * - More modular: separated logic into smaller, focused methods.
 * - Enhanced readability and commented steps.
 * - Reduced code duplication in neighbor and cost calculations.
 */
public class Pathfinder {
    private final CollisionMap collisionMap;
    private final Map<Integer, List<Transportation>> shortcutMap;

    private int startPackedPoint;
    private Map<Integer, Node> allNodes;

    public Pathfinder(CollisionMap collisionMap, Map<Integer, List<Transportation>> shortcutMap) {
        this.collisionMap = collisionMap;
        this.shortcutMap = shortcutMap;
    }

    /**
     * Finds a path from start to end using A*, returning a list of PathSteps.
     * Includes support for "shortcuts" (transportations) which can bypass normal tile restrictions.
     * Times out after 2 seconds if no path is found.
     */
    public List<PathStep> findPath(WorldPoint start, WorldPoint end) {
        Instant startTime = Instant.now();
        Instant deadline = startTime.plusMillis(2000); // 2-second timeout

        PriorityQueue<Node> openSet = new PriorityQueue<>();
        allNodes = new HashMap<>();
        Set<Integer> closedSet = new HashSet<>();

        int startPacked = PointUtils.packWorldPoint(start.getX(), start.getY(), start.getPlane());
        int endPacked = PointUtils.packWorldPoint(end.getX(), end.getY(), end.getPlane());

        this.startPackedPoint = startPacked;

        Node startNode = new Node(startPacked, null, 0, heuristic(startPacked, endPacked), false, null);
        openSet.add(startNode);
        allNodes.put(startPacked, startNode);

        while (!openSet.isEmpty()) {
            if (isTimeUp(deadline)) {
                return Collections.emptyList();
            }

            Node current = openSet.poll();
            if (current == null) break; // No more nodes

            if (closedSet.contains(current.packedPoint)) {
                // Already processed a better route
                continue;
            }

            if (current.packedPoint == endPacked) {
                return constructPath(current);
            }

            closedSet.add(current.packedPoint);

            List<Integer> neighbors = getNeighbors(current.packedPoint);
            for (int neighborPacked : neighbors) {
                if (isTimeUp(deadline)) {
                    return Collections.emptyList();
                }

                if (closedSet.contains(neighborPacked)) {
                    continue;
                }

                processNeighbor(openSet, current, neighborPacked, endPacked);
            }
        }

        // No path found
        return Collections.emptyList();
    }

    /**
     * Processes a neighbor node to see if a better route exists through the current node.
     * Updates openSet and allNodes accordingly.
     */
    private void processNeighbor(PriorityQueue<Node> openSet, Node current, int neighborPacked, int endPacked) {
        if (neighborPacked == current.packedPoint) return;

        boolean isShortcut = isShortcutMovement(current.packedPoint, neighborPacked);
        double movementCost = getMovementCost(current.packedPoint, neighborPacked);
        if (movementCost < 0) return;

        double tentativeG = current.gCost + movementCost;
        Node neighborNode = allNodes.getOrDefault(neighborPacked, new Node(neighborPacked, null, Double.MAX_VALUE, 0, false, null));

        if (tentativeG < neighborNode.gCost) {
            Transportation transportationUsed = null;
            if (isShortcut) {
                transportationUsed = getTransportation(current.packedPoint, neighborPacked);
            }

            neighborNode.cameFrom = current;
            neighborNode.gCost = tentativeG;
            neighborNode.fCost = tentativeG + heuristic(neighborPacked, endPacked);
            neighborNode.viaTransportation = isShortcut;
            neighborNode.transportationUsed = transportationUsed;

            openSet.add(neighborNode);
            allNodes.put(neighborPacked, neighborNode);
        }
    }

    /**
     * Constructs the path by tracing from the end node back to the start node.
     */
    private List<PathStep> constructPath(Node current) {
        List<PathStep> path = new ArrayList<>();
        Node nextNode = null;

        while (current != null) {
            WorldPoint wp = PointUtils.unpackWorldPoint(current.packedPoint);

            Transportation sourceTransportation = current.transportationUsed;
            Transportation destinationTransportation = (nextNode != null) ? nextNode.transportationUsed : null;

            boolean isTransportation = destinationTransportation != null;
            boolean viaTransportation = current.viaTransportation;

            path.add(new PathStep(wp, isTransportation, viaTransportation, sourceTransportation, destinationTransportation));

            nextNode = current;
            current = current.cameFrom;
        }

        Collections.reverse(path);
        return path;
    }

    /**
     * Calculates the movement cost from one tile to another. 
     * Returns -1 if unreachable (except if a shortcut).
     */
    private double getMovementCost(int currentPacked, int neighborPacked) {
        boolean isShortcut = isShortcutMovement(currentPacked, neighborPacked);

        // Shortcut cost can override normal blocked checks.
        double cost = isShortcut ? getShortcutCost(currentPacked, neighborPacked) : -1;
        if (isShortcut && cost >= 0) {
            return cost;
        }

        // If not a shortcut, check normal movement conditions
        if (!canMove(currentPacked, neighborPacked)) {
            return -1;
        }

        // If reachable by normal movement, use distance cost
        return distance(currentPacked, neighborPacked);
    }

    /**
     * Retrieve a transportation object if a shortcut is used.
     */
    private Transportation getTransportation(int currentPacked, int neighborPacked) {
        // Check specific shortcuts from currentPacked
        Transportation t = findTransportationInList(shortcutMap.getOrDefault(currentPacked, Collections.emptyList()), neighborPacked);
        if (t != null) return t;

        // Check global shortcuts if at the start tile
        if (currentPacked == startPackedPoint) {
            t = findTransportationInList(shortcutMap.getOrDefault(PiggyWalkerPlugin.GLOBAL_KEY, Collections.emptyList()), neighborPacked);
        }

        return t;
    }

    private Transportation findTransportationInList(List<Transportation> transports, int neighborPacked) {
        for (Transportation trans : transports) {
            if (trans.isAvailable() && isMatchingDestination(trans, neighborPacked)) {
                return trans;
            }
        }
        return null;
    }

    /**
     * Checks if movement between two tiles involves a shortcut.
     */
    private boolean isShortcutMovement(int currentPacked, int neighborPacked) {
        return hasSpecificShortcut(currentPacked, neighborPacked) ||
               (currentPacked == startPackedPoint && hasGlobalShortcut(neighborPacked));
    }

    private boolean hasSpecificShortcut(int currentPacked, int neighborPacked) {
        return shortcutMap.getOrDefault(currentPacked, Collections.emptyList()).stream()
                .anyMatch(s -> s.isAvailable() && isMatchingDestination(s, neighborPacked));
    }

    private boolean hasGlobalShortcut(int neighborPacked) {
        return shortcutMap.getOrDefault(PiggyWalkerPlugin.GLOBAL_KEY, Collections.emptyList()).stream()
                .anyMatch(s -> s.isAvailable() && isMatchingDestination(s, neighborPacked));
    }

    private boolean isMatchingDestination(Transportation shortcut, int packedPoint) {
        return PointUtils.packWorldPoint(shortcut.getDestinationX(), shortcut.getDestinationY(), shortcut.getDestinationZ()) == packedPoint;
    }

    /**
     * Get the cost of a shortcut if available, else -1.
     */
    private double getShortcutCost(int currentPacked, int neighborPacked) {
        double cost = findMinShortcutCost(shortcutMap.get(currentPacked), neighborPacked);
        if (currentPacked == startPackedPoint) {
            double globalCost = findMinShortcutCost(shortcutMap.get(PiggyWalkerPlugin.GLOBAL_KEY), neighborPacked);
            cost = Math.min(cost < 0 ? Double.MAX_VALUE : cost, globalCost < 0 ? Double.MAX_VALUE : globalCost);
        }
        return (cost == Double.MAX_VALUE) ? -1 : cost;
    }

    private double findMinShortcutCost(List<Transportation> shortcuts, int neighborPacked) {
        if (shortcuts == null || shortcuts.isEmpty()) return -1;
        return shortcuts.stream()
                .filter(s -> isMatchingDestination(s, neighborPacked))
                .mapToDouble(Transportation::getDuration)
                .min()
                .orElse(-1);
    }

    /**
     * Checks if a tile is blocked. If it has a shortcut, it's not considered blocked.
     */
    private boolean isBlocked(int packedPoint) {
        if (packedPoint == startPackedPoint) return false;
        if (isShortcutAvailableAt(packedPoint)) return false;

        return collisionMap.all(
                (short) PointUtils.unpackWorldX(packedPoint),
                (short) PointUtils.unpackWorldY(packedPoint),
                (byte) PointUtils.unpackWorldPlane(packedPoint)
        ) == Flags.NONE;
    }

    private boolean isShortcutAvailableAt(int packedPoint) {
        List<Transportation> list = shortcutMap.get(packedPoint);
        return list != null && !list.isEmpty();
    }

    /**
     * Determines if movement from currentPacked to neighborPacked is possible under normal conditions.
     */
    private boolean canMove(int currentPacked, int neighborPacked) {
        // If at start or came from a transportation, ignore normal block checks.
        Node currentNode = allNodes.get(currentPacked);
        if (currentPacked == startPackedPoint || (currentNode != null && currentNode.viaTransportation)) {
            return true;
        }

        // If a shortcut is used or available at the destination, movement is possible
        if (isShortcutMovement(currentPacked, neighborPacked) || isShortcutAvailableAt(neighborPacked)) {
            return true;
        }

        // Check directional collision flags for normal movement
        Optional<Direction> optDir = Direction.getDirection(
                PointUtils.unpackWorldX(neighborPacked) - PointUtils.unpackWorldX(currentPacked),
                PointUtils.unpackWorldY(neighborPacked) - PointUtils.unpackWorldY(currentPacked)
        );
        if (optDir.isEmpty()) return false;

        return isDirectionUnblocked(currentPacked, neighborPacked, optDir.get());
    }

    private boolean isDirectionUnblocked(int currentPacked, int neighborPacked, Direction direction) {
        byte currentFlag = collisionMap.all(
                (short) PointUtils.unpackWorldX(currentPacked),
                (short) PointUtils.unpackWorldY(currentPacked),
                (byte) PointUtils.unpackWorldPlane(currentPacked)
        );
        byte neighborFlag = collisionMap.all(
                (short) PointUtils.unpackWorldX(neighborPacked),
                (short) PointUtils.unpackWorldY(neighborPacked),
                (byte) PointUtils.unpackWorldPlane(neighborPacked)
        );
        return (currentFlag & direction.getCurrentFlag()) != 0 && (neighborFlag & direction.getNeighborFlag()) != 0;
    }

    /**
     * Euclidean/diagonal distance approximation.
     * Diagonal step costs ~1.414, straight steps cost 1.0.
     */
    private double distance(int aPacked, int bPacked) {
        int dx = Math.abs(PointUtils.unpackWorldX(aPacked) - PointUtils.unpackWorldX(bPacked));
        int dy = Math.abs(PointUtils.unpackWorldY(aPacked) - PointUtils.unpackWorldY(bPacked));
        return (dx == 1 && dy == 1) ? Math.sqrt(2) : 1.0;
    }

    /**
     * Heuristic: Chebyshev distance (max(dx, dy)).
     * Suited for grids where diagonal movement is allowed.
     */
    private double heuristic(int aPacked, int bPacked) {
        int dx = Math.abs(PointUtils.unpackWorldX(aPacked) - PointUtils.unpackWorldX(bPacked));
        int dy = Math.abs(PointUtils.unpackWorldY(aPacked) - PointUtils.unpackWorldY(bPacked));
        return Math.max(dx, dy);
    }

    private boolean isTimeUp(Instant deadline) {
        return Instant.now().isAfter(deadline);
    }

    /**
     * Returns all neighbors of a given tile, including shortcut destinations.
     */
    private List<Integer> getNeighbors(int packedPoint) {
        List<Integer> neighbors = new ArrayList<>();
        Node currentNode = allNodes.get(packedPoint);
        boolean viaTransportation = currentNode != null && currentNode.viaTransportation;

        int x = PointUtils.unpackWorldX(packedPoint);
        int y = PointUtils.unpackWorldY(packedPoint);
        int z = PointUtils.unpackWorldPlane(packedPoint);

        byte flags = collisionMap.all((short) x, (short) y, (byte) z);

        // If via transportation or at start, ignore collision flags and consider all directions
        if (viaTransportation || packedPoint == startPackedPoint) {
            addAllDirectionalNeighbors(neighbors, x, y, z);
        } else {
            addFlagBasedNeighbors(neighbors, x, y, z, flags);
        }

        addTransportationNeighbors(neighbors, packedPoint);
        addAdjacentTransportationTiles(neighbors, x, y, z);

        return neighbors;
    }

    /**
     * Add all eight directions regardless of flags.
     */
    private void addAllDirectionalNeighbors(List<Integer> neighbors, int x, int y, int z) {
        int[][] directions = {
                {0,1}, {0,-1}, {1,0}, {-1,0},
                {1,1}, {-1,1}, {1,-1}, {-1,-1}
        };

        for (int[] dir : directions) {
            neighbors.add(PointUtils.packWorldPoint(x + dir[0], y + dir[1], z));
        }
    }

    /**
     * Add neighbors based on collision flags.
     */
    private void addFlagBasedNeighbors(List<Integer> neighbors, int x, int y, int z, byte flags) {
        // Straight moves
        if ((flags & Flags.NORTH) != 0) neighbors.add(PointUtils.packWorldPoint(x, y + 1, z));
        if ((flags & Flags.SOUTH) != 0) neighbors.add(PointUtils.packWorldPoint(x, y - 1, z));
        if ((flags & Flags.EAST) != 0)  neighbors.add(PointUtils.packWorldPoint(x + 1, y, z));
        if ((flags & Flags.WEST) != 0)  neighbors.add(PointUtils.packWorldPoint(x - 1, y, z));

        // Diagonals
        if ((flags & Flags.NORTHEAST) != 0) neighbors.add(PointUtils.packWorldPoint(x + 1, y + 1, z));
        if ((flags & Flags.NORTHWEST) != 0) neighbors.add(PointUtils.packWorldPoint(x - 1, y + 1, z));
        if ((flags & Flags.SOUTHEAST) != 0) neighbors.add(PointUtils.packWorldPoint(x + 1, y - 1, z));
        if ((flags & Flags.SOUTHWEST) != 0) neighbors.add(PointUtils.packWorldPoint(x - 1, y - 1, z));
    }

    /**
     * Add any transportation shortcuts available from this tile.
     */
    private void addTransportationNeighbors(List<Integer> neighbors, int packedPoint) {
        // Specific shortcuts
        addShortcutDestinations(neighbors, shortcutMap.getOrDefault(packedPoint, Collections.emptyList()), packedPoint);

        // Global shortcuts if at start
        if (packedPoint == startPackedPoint) {
            addShortcutDestinations(neighbors, shortcutMap.getOrDefault(PiggyWalkerPlugin.GLOBAL_KEY, Collections.emptyList()), packedPoint);
        }
    }

    private void addShortcutDestinations(List<Integer> neighbors, List<Transportation> transports, int currentPacked) {
        for (Transportation t : transports) {
            if (t.isAvailable()) {
                int dest = PointUtils.packWorldPoint(t.getDestinationX(), t.getDestinationY(), t.getDestinationZ());
                if (dest != currentPacked) {
                    neighbors.add(dest);
                }
            }
        }
    }

    /**
     * Adjacent tiles that contain shortcuts should be considered reachable neighbors,
     * because we can move onto a tile that hosts a shortcut.
     */
    private void addAdjacentTransportationTiles(List<Integer> neighbors, int x, int y, int z) {
        int[][] directions = {
            {0, 1}, {1, 0}, {0, -1}, {-1, 0},
            {1, 1}, {-1, 1}, {1, -1}, {-1, -1}
        };

        for (int[] dir : directions) {
            int nx = x + dir[0];
            int ny = y + dir[1];
            int neighborPacked = PointUtils.packWorldPoint(nx, ny, z);
            if (isShortcutAvailableAt(neighborPacked)) {
                neighbors.add(neighborPacked);
            }
        }
    }

    /**
     * Internal class representing a node in the search.
     */
    private static class Node implements Comparable<Node> {
        int packedPoint;
        Node cameFrom;
        double gCost;
        double fCost;
        boolean viaTransportation;
        Transportation transportationUsed;

        Node(int packedPoint, Node cameFrom, double gCost, double fCost, boolean viaTransportation, Transportation transportationUsed) {
            this.packedPoint = packedPoint;
            this.cameFrom = cameFrom;
            this.gCost = gCost;
            this.fCost = fCost;
            this.viaTransportation = viaTransportation;
            this.transportationUsed = transportationUsed;
        }

        @Override
        public int compareTo(Node other) {
            return Double.compare(this.fCost, other.fCost);
        }

        @Override
        public int hashCode() {
            return Integer.hashCode(packedPoint);
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (!(obj instanceof Node)) return false;
            Node other = (Node) obj;
            return this.packedPoint == other.packedPoint;
        }
    }
}
