import json
from src.environment.airspace import Airspace
from src.environment.map import GridMap

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