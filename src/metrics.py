import numpy as np
import csv

# metrics for testing and evaluation
class Metrics:
    def __init__(self):
        self.jobs_requested = 0
        self.completed_deliveries = 0
        self.delivery_times = []  
        self.return_times = []  
        self.total_violations = 0
        self.min_separation_observed = float('inf') 
        self.flight_paths = {} 

    def record_path(self, uav_id, pos):
        if uav_id not in self.flight_paths: 
            self.flight_paths[uav_id] = []
        self.flight_paths[uav_id].append(pos.copy())

    def record_delivery_phase(self, duration):
        self.delivery_times.append(duration)

    def record_return_phase(self, duration):
        self.return_times.append(duration)

    def record_separation(self, distance):
        if distance < self.min_separation_observed:
            self.min_separation_observed = distance

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
            "total violations": self.total_violations,
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