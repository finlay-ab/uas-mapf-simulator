from src.config import SimConfig
from src.simulation import Simulation

if __name__ == "__main__":
    # load default config
    cfg = SimConfig()
    
    # override config 
    # cfg.policy = "vo"    
    # cfg.fleet_size = 4      
    # cfg.csv_file = "run_results.csv"

    # run sim
    sim = Simulation(cfg)
    sim.run()
    
    # out
    print(f"\nExperiment Results:")
    stats = sim.metrics.get_summary_statistics()

    for key, value in stats.items():
        print(f"{key}: {value}")

    # save metrics to csv
    sim.metrics.save_to_csv("run_results.csv")