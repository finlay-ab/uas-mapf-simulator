import numpy as np
import csv
import logging
from src.physics import GlobalPosition, LocalPosition

log = logging.getLogger("UAS_Sim")

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
        # depot tracking
        self.depot_stats = {}
        # airspace tracking
        self.airspace_stats = {}
        # job routing between airspaces
        self.job_routing = {}
        # spawn decisions (intra vs inter)
        self.spawn_statistics = {}
        # job lifecycle
        self.job_lifecycle = {}
        # uav-uav collision metrics
        self.min_separation_observed = float('inf')
        # collision events
        self.collision_events = []
        # safety violations
        self.safety_violation_events = []  # list of (uav_id_a, uav_id_b, distance)

        # reservation metrics
        self.reservation_attempts = 0
        self.reservations_granted = 0
        self.reservations_denied = 0

    def record_collision(self, distance, uav_id_a=None, uav_id_b=None):
        self.collision_events.append((uav_id_a, uav_id_b, float(distance)))

    def record_safety_violation(self, distance, uav_id_a=None, uav_id_b=None):
        self.safety_violation_events.append((uav_id_a, uav_id_b, float(distance)))

    def record_separation(self, distance):
        if distance < self.min_separation_observed:
            self.min_separation_observed = distance

    def record_reservation_attempt(self, granted: bool):
        self.reservation_attempts += 1
        if granted:
            self.reservations_granted += 1
        else:
            self.reservations_denied += 1

    # per-depot and per-airspace job tracking
    def record_job_request_at_depot(self, depot_id, origin_airspace, dest_airspace, job_id, creation_time):
        self.jobs_requested += 1
        spawn_type = "INTRA" if origin_airspace == dest_airspace else "INTER"
        # init spawn stats if needed
        if depot_id not in self.spawn_statistics:
            self.spawn_statistics[depot_id] = {'intra': 0, 'inter': 0}
        self.spawn_statistics[depot_id][spawn_type.lower()] += 1
        # init depot stats if needed
        if depot_id not in self.depot_stats:
            self.depot_stats[depot_id] = {
                'spawned': 0, 'completed': 0, 'failed': 0,
                'delivery_times': [], 'return_times': []
            }
        # init airspace stats if needed
        if origin_airspace not in self.airspace_stats:
            self.airspace_stats[origin_airspace] = {
                'total_jobs': 0, 'completed': 0, 'failed': 0,
                'intra_airspace': 0, 'outbound': 0, 'inbound': 0
            }
        if dest_airspace not in self.airspace_stats:
            self.airspace_stats[dest_airspace] = {
                'total_jobs': 0, 'completed': 0, 'failed': 0,
                'intra_airspace': 0, 'outbound': 0, 'inbound': 0
            }
        # track at depot
        self.depot_stats[depot_id]['spawned'] += 1
        # track routing
        if origin_airspace not in self.job_routing:
            self.job_routing[origin_airspace] = {}
        if dest_airspace not in self.job_routing[origin_airspace]:
            self.job_routing[origin_airspace][dest_airspace] = 0
        self.job_routing[origin_airspace][dest_airspace] += 1
        # track at airspace
        self.airspace_stats[origin_airspace]['total_jobs'] += 1
        if origin_airspace == dest_airspace:
            self.airspace_stats[origin_airspace]['intra_airspace'] += 1
        else:
            self.airspace_stats[origin_airspace]['outbound'] += 1
            self.airspace_stats[dest_airspace]['inbound'] += 1
        # track job lifecycle
        self.job_lifecycle[job_id] = {
            'depot_id': depot_id,
            'origin_airspace': origin_airspace,
            'dest_airspace': dest_airspace,
            'created_time': creation_time,
            'started_time': None,
            'completed_time': None,
            'status': 'PENDING'
        }

    def record_job_in_progress_at_depot(self, job_id, started_time):
        if job_id in self.job_lifecycle:
            self.job_lifecycle[job_id]['started_time'] = started_time
            self.job_lifecycle[job_id]['status'] = 'IN_PROGRESS'
            self.in_progress_deliveries += 1

    def record_delivery_complete_at_depot(self, depot_id, job_id, delivery_time, return_time=None, completed_time=None):
        self.completed_deliveries += 1
        self.delivery_times.append(delivery_time)
        if return_time is not None:
            self.return_times.append(return_time)
        if depot_id in self.depot_stats:
            self.depot_stats[depot_id]['completed'] += 1
            self.depot_stats[depot_id]['delivery_times'].append(delivery_time)
            if return_time is not None:
                self.depot_stats[depot_id]['return_times'].append(return_time)
        if job_id in self.job_lifecycle:
            origin_airspace = self.job_lifecycle[job_id]['origin_airspace']
            if origin_airspace in self.airspace_stats:
                self.airspace_stats[origin_airspace]['completed'] += 1
            self.job_lifecycle[job_id]['completed_time'] = completed_time
            self.job_lifecycle[job_id]['status'] = 'COMPLETED'
        if self.in_progress_deliveries > 0:
            self.in_progress_deliveries -= 1

    def record_job_failed_at_depot(self, depot_id, job_id, reason, failed_time):
        self.failed_deliveries += 1
        if depot_id in self.depot_stats:
            self.depot_stats[depot_id]['failed'] += 1
        if job_id in self.job_lifecycle:
            origin_airspace = self.job_lifecycle[job_id]['origin_airspace']
            if origin_airspace in self.airspace_stats:
                self.airspace_stats[origin_airspace]['failed'] += 1
            self.job_lifecycle[job_id]['completed_time'] = failed_time
            self.job_lifecycle[job_id]['status'] = 'FAILED'
            self.job_lifecycle[job_id]['failure_reason'] = reason
        if self.in_progress_deliveries > 0:
            self.in_progress_deliveries -= 1
        log.warning("job %s failed at depot %s (%s)", job_id, depot_id, reason)

    def get_depot_statistics(self):
        depot_stats_summary = {}
        for depot_id, stats in self.depot_stats.items():
            avg_delivery = sum(stats['delivery_times']) / len(stats['delivery_times']) if len(stats['delivery_times']) > 0 else 0.0
            avg_return = sum(stats['return_times']) / len(stats['return_times']) if len(stats['return_times']) > 0 else 0.0
            depot_stats_summary[depot_id] = {
                'spawned': stats['spawned'],
                'completed': stats['completed'],
                'failed': stats['failed'],
                'completion_rate': stats['completed'] / max(stats['spawned'], 1),
                'avg_delivery_time': round(avg_delivery, 2),
                'avg_return_time': round(avg_return, 2),
            }
        return depot_stats_summary

    def get_airspace_statistics(self):
        airspace_stats_summary = {}
        for airspace_id, stats in self.airspace_stats.items():
            completion_rate = stats['completed'] / max(stats['total_jobs'], 1)
            airspace_stats_summary[airspace_id] = {
                'total_jobs': stats['total_jobs'],
                'completed': stats['completed'],
                'failed': stats['failed'],
                'intra_airspace': stats['intra_airspace'],
                'outbound': stats['outbound'],
                'inbound': stats['inbound'],
                'completion_rate': round(completion_rate, 3),
            }
        return airspace_stats_summary

    def get_routing_matrix(self):
        return self.job_routing

    def get_spawn_statistics(self):
        spawn_stats_summary = {}
        for depot_id, stats in self.spawn_statistics.items():
            total = stats['intra'] + stats['inter']
            inter_rate = stats['inter'] / total if total > 0 else 0.0
            spawn_stats_summary[depot_id] = {
                'spawned': total,
                'intra_airspace': stats['intra'],
                'inter_airspace': stats['inter'],
                'inter_airspace_rate': round(inter_rate, 3),
            }
        return spawn_stats_summary

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

        # completion rate over all requested jobs
        if self.jobs_requested > 0:
            completion_rate = round(self.completed_deliveries / self.jobs_requested, 3)
        else:
            completion_rate = 0.0

        # dict 
        return {
            "total jobs requested": self.jobs_requested,
            "completed deliveries": self.completed_deliveries,
            "failed deliveries": self.failed_deliveries,
            "completion rate": completion_rate,
            "avg delivery time (s)": round(avg_delivery, 2),
            "avg return time (s)": round(avg_return, 2),
            "total safety violations": len(self.safety_violation_events),
            "total collisions": len(self.collision_events),
            "min separation (m)": separation_text,
            "reservation attempts": self.reservation_attempts,
            "reservations granted": self.reservations_granted,
            "reservations denied": self.reservations_denied,
        }

    def save_to_csv(self, filename="run_results.csv"):
        # get stats
        stats = self.get_summary_statistics()

        # open file 
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            
            # write data 
            writer.writerow(["SECTION:", "RAW MISSION DATA"])
            writer.writerow(["Mission_ID", "Job_ID", "Origin_Airspace", "Dest_Airspace", "Status", "Delivery_Time_s"])

            # for each job
            mission_id = 0
            for job_id, info in self.job_lifecycle.items():
                if info['status'] not in ('COMPLETED', 'FAILED'):
                    continue
                mission_id += 1
                if info['status'] == 'COMPLETED' and info.get('started_time') is not None and info.get('completed_time') is not None:
                    d_time = info['completed_time'] - info['started_time']
                else:
                    d_time = "N/A"
                writer.writerow([mission_id, job_id, info['origin_airspace'], info['dest_airspace'], info['status'], d_time])

            # add empty lines to separate sections
            writer.writerow([])
            writer.writerow([])

            # write aggregate stats
            writer.writerow(["SECTION:", "AGGREGATE STATISTICS"])
            # loop through dict
            for key, value in stats.items():
                writer.writerow([key, value])