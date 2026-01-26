# UAS MAPF Simulator

A discrete event driven simulation frameowrk for evaluating multi-agent pathfinding (MAPF) based airspace management policies for unmanned aerial systmes (UAS) in urban environments. 

## Objectives 

- To design and implement a discrete-event simulation framework capable of modelling an urban airspace with one or more Airspace Service Providers (ASPs) and processing dynamic UAS flight plan requests.

- To establish and implement a clear set of performance metrics to evaluate the safety and efficiency of management policies, primarily focusing on separation violations (safety) and flight plan timeliness (efficiency).

- To implement a baseline management policy (e.g., a simple first-come, first-served or a basic greedy pathfinder) to serve as a benchmark for comparison.

- To research and implement at least two advanced Multi-Agent Pathfinding (MAPF) algorithm from academic literature as an alternative, optimised management policy within the simulation.

- To design and execute a series of simulation experiments to  compare the performance of the baseline and advanced policies under different levels of UASs limitations and challenges, such as conserving charge, no flight corridors and larger number of UASs.

## Assumptions 

- No low level flight physics
