"""
Run experiments comparing Baseline Heuristics (Nearest Neighbor, Clarke-Wright Savings)
and MILP Formulation on test instances.
"""

import glob
import json
import os
import sys
import time

from models.heuristics import Heuristic2DCVRP
from models.milp_pulp import PulpModel2DCVRP


def load_instance(filepath: str):
    with open(filepath, "r", encoding="utf-8") as f:
        return json.load(f)


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    instances_dir = os.path.join(base_dir, "data", "instances")
    instance_files = sorted(glob.glob(os.path.join(instances_dir, "*.json")))

    if not instance_files:
        print("No instance files found. Generating them first...")
        from data.instance_generator import generate_instance, save_instance_to_file
        for size in [5, 10, 15]:
            inst = generate_instance(num_customers=size, seed=100 + size)
            save_instance_to_file(inst, os.path.join(instances_dir, f"instance_n{size}.json"))
        instance_files = sorted(glob.glob(os.path.join(instances_dir, "*.json")))

    print("=" * 75)
    print(" 2D-CVRP EXPERIMENTAL BENCHMARK")
    print(" Two-Dimensional Capacitated Vehicle Routing Problem")
    print("=" * 75)

    results_table = []

    for fpath in instance_files:
        inst = load_instance(fpath)
        name = inst["instance_name"]
        n_cust = inst["num_customers"]
        print(f"\n>>> Running on Instance: {name} ({n_cust} customers)")

        # 1. Heuristics
        heur = Heuristic2DCVRP(inst)

        t0 = time.time()
        nn_res = heur.nearest_neighbor()
        t_nn = time.time() - t0

        t0 = time.time()
        nn2_res = heur.nearest_neighbor_2opt()
        t_nn2 = time.time() - t0

        t0 = time.time()
        cw_res = heur.clarke_wright_savings()
        t_cw = time.time() - t0

        print(f"  [NN]           Cost: {nn_res['total_cost']:>8.2f} | Vehicles: {nn_res['num_vehicles']} | Time: {t_nn:.4f}s")
        print(f"  [NN + 2-opt]   Cost: {nn2_res['total_cost']:>8.2f} | Vehicles: {nn2_res['num_vehicles']} | Time: {t_nn2:.4f}s")
        print(f"  [CW Savings]   Cost: {cw_res['total_cost']:>8.2f} | Vehicles: {cw_res['num_vehicles']} | Time: {t_cw:.4f}s")

        # 2. Exact MILP Model (for smaller instances n <= 10 or with small time limit)
        milp_res = None
        if n_cust <= 10:
            print("  Solving MILP with PuLP (MTZ formulation)...")
            milp = PulpModel2DCVRP(inst)
            milp_res = milp.solve(time_limit_sec=60)
            if milp_res["total_cost"] is not None:
                print(f"  [MILP MTZ]     Cost: {milp_res['total_cost']:>8.2f} | Status: {milp_res['status']} | Time: {milp_res['runtime_sec']:.2f}s")
            else:
                print(f"  [MILP MTZ]     Status: {milp_res['status']} (PuLP solver not available or timed out)")
        else:
            print("  [MILP MTZ]     Skipped (size > 10; use exact solver for detailed runs)")

    print("\n" + "=" * 75)
    print(" Benchmark completed successfully.")
    print("=" * 75)


if __name__ == "__main__":
    main()
