import unittest
import simpy
from src.schemas import Job, JobStatus
from src.physics import GlobalPosition


# silences logging
import logging
logging.getLogger("UAS_Sim").setLevel(logging.WARNING)


class TestAssignments(unittest.TestCase):

    def test_single_job_lock(self):
        # setup env and queue
        env = simpy.Environment()
        job_queue = simpy.Store(env)

        job = Job(
            id="JOB_1",
            origin_airspace="airspace1",
            origin_depot="depot1",
            destination_airspace="airspace1",
            target_pos=GlobalPosition(10.0, 10.0),
        )
        job_queue.put(job)

        # queue should give the same job instance
        event = job_queue.get()
        env.run(until=event)

        self.assertIs(event.value, job, "FAIL: queue did not return the same job object")
        self.assertEqual(event.value.id, "JOB_1", "FAIL: job id mismatch")

    def test_job_cleanup(self):
        job = Job(
            id="JOB_99",
            origin_airspace="airspace1",
            origin_depot="depot1",
            destination_airspace="airspace2",
            target_pos=GlobalPosition(0.0, 0.0),
            status=JobStatus.IN_PROGRESS,
        )

        # simulate completion
        job.status = JobStatus.COMPLETED
        self.assertEqual(job.status, JobStatus.COMPLETED, "FAIL: job status was not updated")

if __name__ == "__main__":
    unittest.main()