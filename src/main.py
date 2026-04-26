from src.config import SimConfig
from src.simulation import Simulation

if __name__ == "__main__":
    # load default config
    cfg = SimConfig()
    
    # run sim
    sim = Simulation(cfg)
    sim.run()
    
    # output results
    print(f"\nExperiment Results:")

    # save metrics to csv
    stats = sim.metrics.get_summary_statistics()

    for key, value in stats.items():
        print(f"{key}: {value}")

    sim.metrics.save_to_csv("run_results.csv")