import unittest
import os
import subprocess
import sys

class TestOutputs(unittest.TestCase):
    
    def test_files_exist_and_have_data(self):
        # get file names
        csv_name = "run_results.csv"
        log_name = "simulation.log"

        # delete old csv
        if os.path.exists(csv_name):
            os.remove(csv_name)
        if os.path.exists(log_name):
            os.remove(log_name)

        # run main
        result = subprocess.run(
            [sys.executable, "src/main.py"],
            capture_output=True,
            text=True
        )

        # check if it crashed
        self.assertEqual(result.returncode, 0, f"simulation crashed: {result.stderr}")

        # check for csv
        self.assertTrue(os.path.exists(csv_name), f"FAIL: {csv_name} is missing")
        
        # check if csv contains the minimum data
        with open(csv_name, 'r') as f:
            first_line = f.readline()
            # check the header
            self.assertIn("SECTION:", first_line, "FAIL: CSV does not start with the correct header")

        # check for log
        self.assertTrue(os.path.exists(log_name), f"FAIL: {log_name} is missing")
        
        # check log size
        log_size = os.path.getsize(log_name)
        self.assertGreater(log_size, 0, "FAIL: The log file is empty!")

        # check for end of simulation
        with open(log_name, 'r') as f:
            log_content = f.read()
            self.assertIn("END OF SIMULATION", log_content, 
                          "FAIL: Log does not contain 'END OF SIMULATION'. did the sim finish?")

if __name__ == "__main__":
    unittest.main()