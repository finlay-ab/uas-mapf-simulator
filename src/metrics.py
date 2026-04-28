import numpy as np
import csv
from src.physics import GlobalPosition, LocalPosition

# metrics for testing and evaluation
class Metrics:
    def __init__(self):
        # job metrics
        self.jobs_requested = 0
        self.completed_deliveries = 0
        self.failed_deliveries = 0
        self.in_progress_deliveries = 0
        self.delivery_times = []  
        self.return_times = []

        # uav-uav collision metrics
        self.min_separation_observed = float('inf')
        self.flight_paths = {}  # uav_id -> list of positions

        # uav id a, uav id b, distance at collision (uav within safety radius and colliding)
        self.collision_events = [] 

        # uav id a, uav id b, distance at violation (uav within safety radius but not colliding)
        self.safety_violation_events = []  # list of (uav_id_a, uav_id_b, distance)

        # uav-obstacle collision metric
        self.obstacle_collision_events = []  # list of (uav_id, obstacle_id, distance)
        self.airspace_violation_events = []  # list of (uav_id, airspace_id, violation_type)
        

 
    def record_job_request(self):
        self.jobs_requested += 1

    def record_path(self, uav_id, pos: GlobalPosition):
        # record position in global coordinates
        if uav_id not in self.flight_paths: 
            self.flight_paths[uav_id] = []
        self.flight_paths[uav_id].append(pos.as_array())

    def record_delivery_phase(self, duration):
        self.completed_deliveries += 1
        self.delivery_times.append(duration)

    def record_return_phase(self, duration):
        self.return_times.append(duration)

    def record_collision(self, distance, uav_id_a=None, uav_id_b=None):
        self.collision_events.append((uav_id_a, uav_id_b, float(distance)))

    def record_safety_violation(self, distance, uav_id_a=None, uav_id_b=None):
        self.safety_violation_events.append((uav_id_a, uav_id_b, float(distance)))

    def record_separation(self, distance):
        if distance < self.min_separation_observed:
            self.min_separation_observed = distance

    def record_obstacle_collision(self, uav_id, obstacle_id, position: GlobalPosition):
        # record obstacle collision with global position
        self.obstacle_collision_events.append((uav_id, obstacle_id, position.as_array() if hasattr(position, 'as_array') else position))

    def record_airspace_violation(self, uav_id, airspace_id, position: GlobalPosition):
        # record airspace violation with global position
        self.airspace_violation_events.append((uav_id, airspace_id, position.as_array() if hasattr(position, 'as_array') else position))

    def get_summary_statistics(self):
        # if there is delivery times then
        avg_delivery = 0.0
        if len(self.delivery_times) > 0:
            total_d_time = sum(self.delivery_times)
            number_of_d = len(self.delivery_times)
            avg_delivery = total_d_time / number_of_d

        # if there is return times then
        avg_return = 0.0
        if len(self.return_times) > 0:
            total_r_time = sum(self.return_times)
            number_of_r = len(self.return_times)
            avg_return = total_r_time / number_of_r

        # prevents crashing with no data
        if self.min_separation_observed == float('inf'):
            separation_text = "N/A"
        else:
            separation_text = round(self.min_separation_observed, 2)

        # dict 
        return {
            "total jobs requested": self.jobs_requested,
            "completed deliveries": self.completed_deliveries,
            "avg delivery time (s)": round(avg_delivery, 2),
            "avg return time (s)": round(avg_return, 2),
            "total safety violations": len(self.safety_violation_events),
            "total collisions": len(self.collision_events),
            "min separation (m)": separation_text
        }

    def save_to_csv(self, filename="run_results.csv"):
        # get stats
        stats = self.get_summary_statistics()

        # open file 
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            
            # write data 
            writer.writerow(["SECTION:", "RAW MISSION DATA"])
            writer.writerow(["Mission_ID", "Delivery_Time_s", "Return_Time_s"])
            
            for i in range(len(self.delivery_times)):
                mission_id = i + 1
                d_time = self.delivery_times[i]
                r_time = "N/A"
                if i < len(self.return_times):
                    r_time = self.return_times[i]
                writer.writerow([mission_id, d_time, r_time])

            # add empty lines to separate sections
            writer.writerow([])
            writer.writerow([])

            # write aggregate stats
            writer.writerow(["SECTION:", "AGGREGATE STATISTICS"])
            # loop through dict
            for key, value in stats.items():
                writer.writerow([key, value])