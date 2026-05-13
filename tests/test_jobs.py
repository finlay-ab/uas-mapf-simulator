import unittest
from src.metrics import Metrics

class TestJobs(unittest.TestCase):

    def _record_job(self, met, job_id):
        met.record_job_request_at_depot("DEPOT_A", "A1", "A1", job_id, 0.0)

    def test_job_counting(self):
        # setup metrics
        met = Metrics()
        
        # request 5 jobs
        for i in range(5):
            self._record_job(met, f"JOB_{i}")
            
        # fail if jobs requested is not 5
        self.assertEqual(met.jobs_requested, 5, f"FAIL: jobs requested should be 5, got {met.jobs_requested}")

    def test_completion_tracking(self):
        # setup metrics
        met = Metrics()
        
        # simulate 3 successful deliveries
        for i, d_time in enumerate([15.0, 20.0, 25.0]):
            job_id = f"JOB_{i}"
            self._record_job(met, job_id)
            met.record_job_in_progress_at_depot(job_id, 1.0)
            met.record_delivery_complete_at_depot("DEPOT_A", job_id, d_time, return_time=2.0, completed_time=3.0)
        
        # check completed count
        self.assertEqual(met.completed_deliveries, 3, "FAIL: completed deliveries not tracked correctly")
        
        # check if list of times has 3 items
        self.assertEqual(len(met.delivery_times), 3, "FAIL: delivery times list length mismatch")

    def test_summary_consistency(self):
        # ensure dictionary matches the data
        met = Metrics()
        self._record_job(met, "JOB_1")
        self._record_job(met, "JOB_2")
        met.record_job_in_progress_at_depot("JOB_1", 1.0)
        met.record_delivery_complete_at_depot("DEPOT_A", "JOB_1", 10.0, completed_time=2.0)
        
        stats = met.get_summary_statistics()
        
        # fail if summary doesn't match internal variables
        self.assertEqual(stats["total jobs requested"], 2, "FAIL: summary total jobs mismatch")
        self.assertEqual(stats["completed deliveries"], 1, "FAIL: summary completed deliveries mismatch")

if __name__ == "__main__":
    unittest.main()