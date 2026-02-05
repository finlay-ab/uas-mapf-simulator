import unittest
from src.metrics import Metrics

class TestJobs(unittest.TestCase):

    def test_job_counting(self):
        # setup metrics
        met = Metrics()
        
        # request 5 jobs
        for i in range(5):
            met.record_job_request()
            
        # fail if jobs requested is not 5
        self.assertEqual(met.jobs_requested, 5, f"FAIL: jobs requested should be 5, got {met.jobs_requested}")

    def test_completion_tracking(self):
        # setup metrics
        met = Metrics()
        
        # simulate 3 successful deliveries
        met.record_delivery_phase(15.0)
        met.record_delivery_phase(20.0)
        met.record_delivery_phase(25.0)
        
        # check completed count
        self.assertEqual(met.completed_deliveries, 3, "FAIL: completed deliveries not tracked correctly")
        
        # check if list of times has 3 items
        self.assertEqual(len(met.delivery_times), 3, "FAIL: delivery times list length mismatch")

    def test_summary_consistency(self):
        # ensure dictionary matches the data
        met = Metrics()
        met.record_job_request()
        met.record_job_request()
        met.record_delivery_phase(10.0)
        
        stats = met.get_summary_statistics()
        
        # fail if summary doesn't match internal variables
        self.assertEqual(stats["total jobs requested"], 2, "FAIL: summary total jobs mismatch")
        self.assertEqual(stats["completed deliveries"], 1, "FAIL: summary completed deliveries mismatch")

if __name__ == "__main__":
    unittest.main()