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
 * A* pathfinding implementation with support for shortcuts/transportations.
 * This refactored version improves readability, uses a closed set,
 * and simplifies time checks and heuristic calculations.
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
            if (current == null) {
                break; 
            }

            if (closedSet.contains(current.packedPoint)) {
                // We've already processed a better route to this tile
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

        return Collections.emptyList();
    }

    /**
     * Process a neighbor node: Update costs and paths if a better route is found.
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
     * Reconstruct the path from the end node by tracing back `cameFrom` links.
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
     * Returns the cost of moving from currentPacked to neighborPacked.
     * If the tile is not reachable (blocked) and not a shortcut, returns -1.
     */
    private double getMovementCost(int currentPacked, int neighborPacked) {
        boolean isShortcut = isShortcutMovement(currentPacked, neighborPacked);
        double cost = getShortcutCost(currentPacked, neighborPacked);
        boolean canMoveTile = canMove(currentPacked, neighborPacked);
        boolean blocked = isBlocked(neighborPacked);

        // If we reached this tile via transportation, we might ignore blockages
        Node currentNode = allNodes.get(currentPacked);
        if (currentNode != null && currentNode.viaTransportation) {
            blocked = false;
        }

        double distance = distance(currentPacked, neighborPacked);
        if (isShortcut) {
            blocked = false; // Shortcut overrides block checks
        }

        // If the tile has a shortcut, consider it accessible
        if (isShortcutAvailableAt(neighborPacked)) {
            blocked = false;
            canMoveTile = true;
        }

        double moveCost = (canMoveTile && !blocked) ? distance : -1;
        return isShortcut ? cost : moveCost;
    }

    private Transportation getTransportation(int currentPacked, int neighborPacked) {
        // Check specific shortcuts
        List<Transportation> transports = shortcutMap.getOrDefault(currentPacked, Collections.emptyList());
        for (Transportation t : transports) {
            if (t.isAvailable() && isMatchingDestination(t, neighborPacked)) {
                return t;
            }
        }

        // Check global shortcuts if at start
        if (currentPacked == startPackedPoint) {
            List<Transportation> globalTransports = shortcutMap.getOrDefault(PiggyWalkerPlugin.GLOBAL_KEY, Collections.emptyList());
            for (Transportation t : globalTransports) {
                if (t.isAvailable() && isMatchingDestination(t, neighborPacked)) {
                    return t;
                }
            }
        }
        return null;
    }

    private boolean isShortcutMovement(int currentPacked, int neighborPacked) {
        return hasSpecificShortcut(currentPacked, neighborPacked) ||
               (currentPacked == startPackedPoint && hasGlobalShortcut(neighborPacked));
    }

    private boolean hasSpecificShortcut(int currentPacked, int neighborPacked) {
        List<Transportation> shortcuts = shortcutMap.getOrDefault(currentPacked, Collections.emptyList());
        return shortcuts.stream().anyMatch(s -> s.isAvailable() && isMatchingDestination(s, neighborPacked));
    }

    private boolean hasGlobalShortcut(int neighborPacked) {
        List<Transportation> globalShortcuts = shortcutMap.getOrDefault(PiggyWalkerPlugin.GLOBAL_KEY, Collections.emptyList());
        return globalShortcuts.stream().anyMatch(s -> s.isAvailable() && isMatchingDestination(s, neighborPacked));
    }

    private boolean isMatchingDestination(Transportation shortcut, int packedPoint) {
        return PointUtils.packWorldPoint(shortcut.getDestinationX(), shortcut.getDestinationY(), shortcut.getDestinationZ()) == packedPoint;
    }

    private double getShortcutCost(int currentPacked, int neighborPacked) {
        double specificCost = getCostFromShortcuts(shortcutMap.get(currentPacked), neighborPacked);
        double globalCost = (currentPacked == startPackedPoint) ? getCostFromShortcuts(shortcutMap.get(PiggyWalkerPlugin.GLOBAL_KEY), neighborPacked) : Double.MAX_VALUE;
        double minCost = Math.min(specificCost, globalCost);
        return minCost < Double.MAX_VALUE ? minCost : -1;
    }

    private double getCostFromShortcuts(List<Transportation> shortcuts, int neighborPacked) {
        if (shortcuts == null) return Double.MAX_VALUE;
        return shortcuts.stream()
                .filter(s -> isMatchingDestination(s, neighborPacked))
                .mapToDouble(Transportation::getDuration)
                .min()
                .orElse(Double.MAX_VALUE);
    }

    private boolean isBlocked(int packedPoint) {
        // Start point or a tile with shortcuts isn't considered blocked.
        if (packedPoint == startPackedPoint) {
            return false;
        }
        if (isShortcutAvailableAt(packedPoint)) {
            return false;
        }
        return collisionMap.all(
                (short) PointUtils.unpackWorldX(packedPoint),
                (short) PointUtils.unpackWorldY(packedPoint),
                (byte) PointUtils.unpackWorldPlane(packedPoint)
        ) == Flags.NONE;
    }

    private boolean isShortcutAvailableAt(int packedPoint) {
        return shortcutMap.containsKey(packedPoint) && !shortcutMap.get(packedPoint).isEmpty();
    }

    private boolean canMove(int currentPacked, int neighborPacked) {
        if (currentPacked == startPackedPoint) {
            return true;
        }

        Node currentNode = allNodes.get(currentPacked);
        if (currentNode != null && currentNode.viaTransportation) {
            return true;
        }

        if (isShortcutMovement(currentPacked, neighborPacked)) return true;
        if (isShortcutAvailableAt(neighborPacked)) return true;

        Optional<Direction> optionalDirection = Direction.getDirection(
                PointUtils.unpackWorldX(neighborPacked) - PointUtils.unpackWorldX(currentPacked),
                PointUtils.unpackWorldY(neighborPacked) - PointUtils.unpackWorldY(currentPacked)
        );
        if (optionalDirection.isEmpty()) return false;

        Direction direction = optionalDirection.get();
        return isDirectionUnblocked(currentPacked, neighborPacked, direction);
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

    private double distance(int aPacked, int bPacked) {
        int dx = Math.abs(PointUtils.unpackWorldX(aPacked) - PointUtils.unpackWorldX(bPacked));
        int dy = Math.abs(PointUtils.unpackWorldY(aPacked) - PointUtils.unpackWorldY(bPacked));
        // Diagonal cost ~1.414 for diagonal moves, else 1.0
        return (dx == 1 && dy == 1) ? Math.sqrt(2) : 1.0;
    }

    /**
     * Heuristic using Chebyshev distance (max(dx, dy)) is often suitable for grids allowing diagonal movement.
     */
    private double heuristic(int aPacked, int bPacked) {
        int dx = Math.abs(PointUtils.unpackWorldX(aPacked) - PointUtils.unpackWorldX(bPacked));
        int dy = Math.abs(PointUtils.unpackWorldY(aPacked) - PointUtils.unpackWorldY(bPacked));
        return Math.max(dx, dy);
    }

    private boolean isTimeUp(Instant deadline) {
        return Instant.now().isAfter(deadline);
    }

    private List<Integer> getNeighbors(int packedPoint) {
        List<Integer> neighbors = new ArrayList<>();
        Node currentNode = allNodes.get(packedPoint);
        boolean viaTransportation = currentNode != null && currentNode.viaTransportation;

        int x = PointUtils.unpackWorldX(packedPoint);
        int y = PointUtils.unpackWorldY(packedPoint);
        int z = PointUtils.unpackWorldPlane(packedPoint);
        byte flags = collisionMap.all((short) x, (short) y, (byte) z);

        // If via transportation or starting tile, ignore flags and add all directions
        if (viaTransportation || packedPoint == startPackedPoint) {
            addCardinalNeighborsIgnoreFlags(neighbors, x, y, z);
        } else {
            addCardinalNeighbors(neighbors, x, y, z, flags);
        }

        addTransportationNeighbors(neighbors, packedPoint);
        addAdjacentTransportationTiles(neighbors, x, y, z);

        return neighbors;
    }

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

    private void addCardinalNeighborsIgnoreFlags(List<Integer> neighbors, int x, int y, int z) {
        neighbors.add(PointUtils.packWorldPoint(x, y + 1, z));
        neighbors.add(PointUtils.packWorldPoint(x, y - 1, z));
        neighbors.add(PointUtils.packWorldPoint(x + 1, y, z));
        neighbors.add(PointUtils.packWorldPoint(x - 1, y, z));
        neighbors.add(PointUtils.packWorldPoint(x + 1, y + 1, z));
        neighbors.add(PointUtils.packWorldPoint(x - 1, y + 1, z));
        neighbors.add(PointUtils.packWorldPoint(x + 1, y - 1, z));
        neighbors.add(PointUtils.packWorldPoint(x - 1, y - 1, z));
    }

    private void addCardinalNeighbors(List<Integer> neighbors, int x, int y, int z, byte flags) {
        if ((flags & Flags.NORTH) != 0) {
            neighbors.add(PointUtils.packWorldPoint(x, y + 1, z));
        }
        if ((flags & Flags.SOUTH) != 0) {
            neighbors.add(PointUtils.packWorldPoint(x, y - 1, z));
        }
        if ((flags & Flags.EAST) != 0) {
            neighbors.add(PointUtils.packWorldPoint(x + 1, y, z));
        }
        if ((flags & Flags.WEST) != 0) {
            neighbors.add(PointUtils.packWorldPoint(x - 1, y, z));
        }
        if ((flags & Flags.NORTHEAST) != 0) {
            neighbors.add(PointUtils.packWorldPoint(x + 1, y + 1, z));
        }
        if ((flags & Flags.NORTHWEST) != 0) {
            neighbors.add(PointUtils.packWorldPoint(x - 1, y + 1, z));
        }
        if ((flags & Flags.SOUTHEAST) != 0) {
            neighbors.add(PointUtils.packWorldPoint(x + 1, y - 1, z));
        }
        if ((flags & Flags.SOUTHWEST) != 0) {
            neighbors.add(PointUtils.packWorldPoint(x - 1, y - 1, z));
        }
    }

    private void addTransportationNeighbors(List<Integer> neighbors, int packedPoint) {
        // Specific shortcuts
        shortcutMap.getOrDefault(packedPoint, Collections.emptyList()).stream()
                .filter(Transportation::isAvailable)
                .map(t -> PointUtils.packWorldPoint(t.getDestinationX(), t.getDestinationY(), t.getDestinationZ()))
                .filter(dest -> dest != packedPoint)
                .forEach(neighbors::add);

        // Global shortcuts if at start
        if (packedPoint == startPackedPoint) {
            shortcutMap.getOrDefault(PiggyWalkerPlugin.GLOBAL_KEY, Collections.emptyList()).stream()
                    .filter(Transportation::isAvailable)
                    .map(t -> PointUtils.packWorldPoint(t.getDestinationX(), t.getDestinationY(), t.getDestinationZ()))
                    .filter(dest -> dest != startPackedPoint)
                    .forEach(neighbors::add);
        }
    }

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

