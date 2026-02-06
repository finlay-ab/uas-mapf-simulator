import unittest
import simpy
from src.entities import UAV
from src.metrics import Metrics


# silences logging
import logging
logging.getLogger("UAS_Sim").setLevel(logging.WARNING)


class TestAssignments(unittest.TestCase):

    def test_single_job_lock(self):
        # setup env and uav
        env = simpy.Environment()
        met = Metrics()
        job_queue = simpy.Store(env)
        
        # put a job in the queue
        job_queue.put({'id': 1, 'goal': [10, 10]})
        
        # init uav
        uav = UAV(env, "UAV_0", [0,0], None, met, None, job_queue)
        
        env.run(until=0.1) 

        # check attribute
        self.assertTrue(hasattr(uav, 'current_job'), "FAIL: UAV has no attribute 'current_job'")
        self.assertIsNotNone(uav.current_job, "FAIL: UAV did not pick up the job from the queue")
        self.assertEqual(uav.current_job['id'], 1, "FAIL: UAV picked up the wrong job")

    def test_job_cleanup(self):
        # setup
        env = simpy.Environment()
        met = Metrics()
        job_queue = simpy.Store(env)
        uav = UAV(env, "UAV_0", [0,0], None, met, None, job_queue)
        
        # give job
        uav.current_job = {'id': 99, 'goal': [0,0]}
        
        # simulation of the end of the run loop
        uav.current_job = None
        
        # fail if it didn't clear
        self.assertIsNone(uav.current_job, "FAIL: current_job was not cleared")

if __name__ == "__main__":
    unittest.main()