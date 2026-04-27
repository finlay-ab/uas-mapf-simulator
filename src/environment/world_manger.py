import json
from src.environment.airspace import Airspace
from src.environment.map import GridMap
from src.physics import GlobalPosition, LocalPosition
from src.entities import UAV
from src.schemas import Waypoint, WayPointType

class WorldManager:
    def __init__(self, config, env, policy):
        self.config = config
        self.env = env
        self.policy = policy
        self.airspaces = {}
        self.graph = {}
        
        # load airspaces
        self._load_airspaces()
        
        # create graph
        self._create_graph()

        
    def _load_airspaces(self):
        # read json
        with open(self.config.world_config, 'r') as f:
            manifest = json.load(f)
        
       # for each airspace in json create airspace object
        for airspace in manifest['airspaces']:
            # get config files 
            airspace_id = airspace['id']
            config_file = airspace['config_file']
            map_file = airspace['map_file']
            fleet_file = airspace['fleet_file']
            
            # create airspace and store in dict
            self.airspaces[airspace_id] = Airspace(airspace_id, config_file, map_file, fleet_file, self.env, self.policy, self.config, world_manager=self)
    
    def _create_graph(self):
        for airspace_id in self.airspaces.keys():
            self.graph[airspace_id] = [] 

    def get_airspace(self, airspace_id):
        return self.airspaces.get(airspace_id, None)
    
    def get_all_airspaces(self):
        return self.airspaces

    def get_path(self, start_airspace_id, end_airspace_id, current_pos: GlobalPosition, target_pos: GlobalPosition) -> list[Waypoint]:
        # for now assume airspaces must be adjacent and return direct path along with rturn back to start
        waypoints = []

        if not (start_airspace_id in self.airspaces and end_airspace_id in self.airspaces):
             return None
        
        # append start 
        waypoints.append(Waypoint(current_pos, WayPointType.TAKEOFF, start_airspace_id))

        if start_airspace_id != end_airspace_id:
            # handover out
            gate_out = self.airspaces[start_airspace_id].map.get_gate_to_airspace(end_airspace_id)
            if gate_out is None:
                raise ValueError(f"No gate from {start_airspace_id} to {end_airspace_id}")

            # convert grid to global position
            gate_out_local_pos = self.airspaces[start_airspace_id].map.grid_to_local(gate_out.position)
            gate_out_pos = self.airspaces[start_airspace_id].local_to_world(gate_out_local_pos)

            # append gate 
            waypoints.append(Waypoint(gate_out_pos, WayPointType.HANDOVER_OUT, start_airspace_id, gate_id=gate_out.id))

            # get target gate
            gate_in = self.airspaces[end_airspace_id].map.get_gate(gate_out.target_gate_id)
            if gate_in is None:
                raise ValueError(f"No gate with id {gate_out.target_gate_id} in airspace {end_airspace_id}")
            gate_in_local_pos = self.airspaces[end_airspace_id].map.grid_to_local(gate_in.position)
            gate_in_pos = self.airspaces[end_airspace_id].local_to_world(gate_in_local_pos)
            # append gate in
            waypoints.append(Waypoint(gate_in_pos, WayPointType.HANDOVER_IN, end_airspace_id, gate_id=gate_in.id))


        # append end
        waypoints.append(Waypoint(target_pos, WayPointType.DELIVERY, end_airspace_id))

        # start return trip

        if start_airspace_id != end_airspace_id:
            # get return gate
            return_gate_out = self.airspaces[end_airspace_id].map.get_gate_to_airspace(start_airspace_id)
            if return_gate_out is None:
                raise ValueError(f"No gate from {end_airspace_id} to {start_airspace_id}")
            return_gate_out_local_pos = self.airspaces[end_airspace_id].map.grid_to_local(return_gate_out.position)
            return_gate_out_pos = self.airspaces[end_airspace_id].local_to_world(return_gate_out_local_pos)
            # append gate
            waypoints.append(Waypoint(return_gate_out_pos, WayPointType.HANDOVER_OUT, end_airspace_id, gate_id=return_gate_out.id))

            # get return gate in
            return_gate_in = self.airspaces[start_airspace_id].map.get_gate(return_gate_out.target_gate_id)
            if return_gate_in is None:
                raise ValueError(f"No gate with id {return_gate_out.target_gate_id} in airspace {start_airspace_id}")
            return_gate_in_local_pos = self.airspaces[start_airspace_id].map.grid_to_local(return_gate_in.position)
            return_gate_in_pos = self.airspaces[start_airspace_id].local_to_world(return_gate_in_local_pos)
            # append gate in
            waypoints.append(Waypoint(return_gate_in_pos, WayPointType.HANDOVER_IN, start_airspace_id, gate_id=return_gate_in.id))

        # append landing 
        waypoints.append(Waypoint(current_pos, WayPointType.LANDING, start_airspace_id))

        return waypoints

    # self.world_manager.complete_handover(from_airspace_id, to_airspace_id, uav_id)
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

    