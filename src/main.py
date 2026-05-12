import argparse

from src.config import SimConfig
from src.simulation import Simulation
import logging

log = logging.getLogger("UAS_Sim")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="run UAS MAPF simulation")
    parser.add_argument("--config", help="load json config file")
    parser.add_argument("--seed", type=int, help="seed")
    args = parser.parse_args()

    # load config from file if provided
    if args.config:
        overrides = {"seed": args.seed} if args.seed is not None else None
        cfg = SimConfig.from_file(args.config, overrides=overrides)
    # else use seed provided
    else:
        cfg = SimConfig()
        cfg.seed = args.seed
    
    # run sim
    sim = Simulation(cfg)
    sim.run()
    
    # output results
    log.info("")
    log.info("="*80)
    log.info("EXPERIMENT RESULTS")
    log.info("="*80)

    # get and display stats
    stats = sim.metrics.get_summary_statistics()
    for key, value in stats.items():
        log.info(f"{key}: {value}")
    
    # display per-depot statistics
    log.info("")
    depot_stats = sim.metrics.get_depot_statistics()
    for depot_id, stats in depot_stats.items():
        log.info(f"Depot {depot_id}: spawned={stats['spawned']}, completed={stats['completed']}, "
                 f"failed={stats['failed']}, completion_rate={stats['completion_rate']}")
    
    # display per-airspace statistics
    log.info("")
    airspace_stats = sim.metrics.get_airspace_statistics()
    for airspace_id, stats in airspace_stats.items():
        log.info(f"Airspace {airspace_id}: total={stats['total_jobs']}, completed={stats['completed']}, "
                 f"intra={stats['intra_airspace']}, outbound={stats['outbound']}, inbound={stats['inbound']}")
    
    # display spawn decision statistics
    log.info("")
    spawn_stats = sim.metrics.get_spawn_statistics()
    for depot_id, stats in spawn_stats.items():
        log.info(f"Depot {depot_id} spawn: intra={stats['intra_airspace']}, "
                 f"inter={stats['inter_airspace']} (rate={stats['inter_airspace_rate']})")
    
    # display job routing matrix
    routing = sim.metrics.get_routing_matrix()
    if routing:
        log.info("")
        for origin, destinations in routing.items():
            for dest, count in destinations.items():
                log.info(f"{origin} -> {dest}: {count} jobs")
    
    log.info("")
    log.info(f"Saving results to {cfg.csv_file}")
    sim.metrics.save_to_csv(cfg.csv_file)