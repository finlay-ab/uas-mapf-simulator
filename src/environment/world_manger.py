import json
import logging
from collections import deque

from src.environment.airspace import Airspace
from src.physics import GlobalPosition
from src.schemas import Waypoint, WayPointType

log = logging.getLogger("UAS_Sim")


class WorldManager:
    def __init__(self, config, env, policy, metrics):
        self.config = config
        self.env = env
        self.policy = policy
        self.metrics = metrics
        self.airspaces = {}
        self.graph = {}

        self.manifest = self._load_manifest()
        self._load_airspaces()
        self._create_graph()

    def _load_manifest(self):
        with open(self.config.world_config, "r") as f:
            return json.load(f)

    def _load_airspaces(self):
        # for each airspace in manifest create airspace object
        for airspace in self.manifest["airspaces"]:
            airspace_id = airspace["id"]
            config_file = airspace["config_file"]
            map_file = airspace["map_file"]
            fleet_file = airspace["fleet_file"]

            self.airspaces[airspace_id] = Airspace(
                airspace_id,
                config_file,
                map_file,
                fleet_file,
                self.env,
                self.policy,
                self.config,
                world_manager=self,
                metrics=self.metrics,
            )
        return self.airspaces

    def _create_graph(self):
        for airspace_id in self.airspaces.keys():
            self.graph[airspace_id] = []

        # build directed graph from manifest connections
        for connection in self.manifest.get("connections", []):
            from_airspace = connection["from_airspace"]
            to_airspace = connection["to_airspace"]

            if from_airspace not in self.graph or to_airspace not in self.graph:
                log.warning(
                    "Skipping invalid connection from %s to %s; airspace missing in manifest",
                    from_airspace,
                    to_airspace,
                )
                continue

            self.graph[from_airspace].append(
                {
                    "to_airspace": to_airspace,
                    "from_gate_id": connection["from_gate_id"],
                    "to_gate_id": connection["to_gate_id"],
                }
            )

    def get_airspace(self, airspace_id):
        return self.airspaces.get(airspace_id, None)

    def get_all_airspaces(self):
        return self.airspaces

    # bfs over the airspace graph. returns the ordered list of edges from start to end
    def _find_airspace_route(self, start_airspace_id, end_airspace_id):
        if start_airspace_id == end_airspace_id:
            return []

        queue = deque([start_airspace_id])
        visited = {start_airspace_id}
        parent = {}

        while queue:
            current = queue.popleft()
            for edge in self.graph.get(current, []):
                nxt = edge["to_airspace"]
                if nxt in visited:
                    continue

                visited.add(nxt)
                parent[nxt] = (current, edge)

                if nxt == end_airspace_id:
                    route_edges = []
                    cursor = end_airspace_id
                    while cursor != start_airspace_id:
                        prev, prev_edge = parent[cursor]
                        route_edges.append(
                            {
                                "from_airspace": prev,
                                "to_airspace": cursor,
                                "from_gate_id": prev_edge["from_gate_id"],
                                "to_gate_id": prev_edge["to_gate_id"],
                            }
                        )
                        cursor = prev
                    route_edges.reverse()
                    return route_edges

                queue.append(nxt)

        return None

    def _global_gate_position(self, airspace_id, gate_id):
        airspace = self.airspaces[airspace_id]
        gate = airspace.map.get_gate(gate_id)
        if gate is None:
            return None, None
        gate_local_pos = airspace.map.grid_to_local(gate.position)
        return gate, airspace.local_to_world(gate_local_pos)

    def _append_handover_waypoints(self, waypoints, route_edges):
        for edge in route_edges:
            gate_out, gate_out_pos = self._global_gate_position(edge["from_airspace"], edge["from_gate_id"])
            gate_in, gate_in_pos = self._global_gate_position(edge["to_airspace"], edge["to_gate_id"])

            if gate_out is None or gate_in is None:
                return False

            waypoints.append(
                Waypoint(
                    gate_out_pos,
                    WayPointType.HANDOVER_OUT,
                    edge["from_airspace"],
                    gate_id=gate_out.id,
                )
            )
            waypoints.append(
                Waypoint(
                    gate_in_pos,
                    WayPointType.HANDOVER_IN,
                    edge["to_airspace"],
                    gate_id=gate_in.id,
                )
            )

        return True

    def get_path(self, start_airspace_id, end_airspace_id, current_pos: GlobalPosition, target_pos: GlobalPosition):
        if not (start_airspace_id in self.airspaces and end_airspace_id in self.airspaces):
            return None

        waypoints = [Waypoint(current_pos, WayPointType.TAKEOFF, start_airspace_id)]

        forward_edges = self._find_airspace_route(start_airspace_id, end_airspace_id)
        if forward_edges is None:
            return None

        if not self._append_handover_waypoints(waypoints, forward_edges):
            return None

        waypoints.append(Waypoint(target_pos, WayPointType.DELIVERY, end_airspace_id))

        return_edges = self._find_airspace_route(end_airspace_id, start_airspace_id)
        if return_edges is None:
            return None

        if not self._append_handover_waypoints(waypoints, return_edges):
            return None

        waypoints.append(Waypoint(current_pos, WayPointType.LANDING, start_airspace_id))

        return waypoints

    def complete_handover(self, from_airspace_id, to_airspace_id, uav_id):
        uav = self.get_airspace(from_airspace_id).get_uav(uav_id)
        if uav is None:
            raise ValueError(f"UAV {uav_id} not found in airspace {from_airspace_id}")

        to_airspace = self.get_airspace(to_airspace_id)
        if to_airspace is None:
            raise ValueError(f"Airspace {to_airspace_id} not found")

        # remove from old airspace
        self.get_airspace(from_airspace_id).remove_uav(uav)

        # add to new airspace
        to_airspace.add_uav(uav)

        # update uav airspace ref
        uav.airspace = to_airspace