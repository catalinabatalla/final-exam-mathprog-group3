"""
Instance Generator for the 2D-CVRP (Two-Dimensional Capacitated Vehicle Routing Problem).
Generates synthetic urban city instances with asymmetric distances, non-proportional weight
and volume demands, and vehicle capacity specifications.
"""

import json
import math
import os
import random
from typing import Dict, Any


def generate_instance(
    num_customers: int,
    seed: int = 42,
    grid_size: float = 100.0,
    vehicle_capacity_weight: float = 100.0,
    vehicle_capacity_volume: float = 100.0,
    asymmetry_factor: float = 0.25,
) -> Dict[str, Any]:
    """
    Generates an instance of the 2D-CVRP.
    
    Parameters:
    - num_customers: Number of customer nodes (excluding depot).
    - seed: Random seed for reproducibility.
    - grid_size: Coordinate space (grid_size x grid_size).
    - vehicle_capacity_weight: Maximum weight capacity Q_W.
    - vehicle_capacity_volume: Maximum volume capacity Q_V.
    - asymmetry_factor: Max percentage variation in distance due to one-way streets.
    """
    random.seed(seed)

    # Node 0 is the depot, centered or placed near the center
    nodes = {
        0: {
            "id": 0,
            "name": "Depot",
            "x": grid_size / 2.0,
            "y": grid_size / 2.0,
            "weight": 0.0,
            "volume": 0.0,
        }
    }

    for i in range(1, num_customers + 1):
        # Generate random coordinates
        x = round(random.uniform(0.0, grid_size), 2)
        y = round(random.uniform(0.0, grid_size), 2)

        # Weight and volume are independent to reflect non-proportionality
        # E.g. some goods are heavy & compact (metal parts), others are light & bulky (foam, textiles)
        weight = round(random.uniform(5.0, 25.0), 2)
        volume = round(random.uniform(5.0, 30.0), 2)

        nodes[i] = {
            "id": i,
            "name": f"Customer_{i}",
            "x": x,
            "y": y,
            "weight": weight,
            "volume": volume,
        }

    # Compute asymmetric distance matrix
    # c_ij can differ from c_ji due to one-way streets and traffic topography
    all_node_ids = list(nodes.keys())
    distance_matrix: Dict[str, Dict[str, float]] = {}

    for i in all_node_ids:
        distance_matrix[str(i)] = {}
        for j in all_node_ids:
            if i == j:
                distance_matrix[str(i)][str(j)] = 0.0
            else:
                euclidean = math.hypot(
                    nodes[i]["x"] - nodes[j]["x"],
                    nodes[i]["y"] - nodes[j]["y"],
                )
                # Apply asymmetric perturbation
                perturbation = random.uniform(1.0 - asymmetry_factor, 1.0 + asymmetry_factor)
                dist = round(euclidean * perturbation, 2)
                distance_matrix[str(i)][str(j)] = dist

    # Upper bound estimate for number of vehicles: at least sum(demands) / capacity
    total_w = sum(nodes[i]["weight"] for i in range(1, num_customers + 1))
    total_v = sum(nodes[i]["volume"] for i in range(1, num_customers + 1))
    min_vehicles = max(
        math.ceil(total_w / vehicle_capacity_weight),
        math.ceil(total_v / vehicle_capacity_volume),
    )
    # Upper bound on fleet size
    max_vehicles = min(num_customers, min_vehicles + 3)

    instance = {
        "instance_name": f"2DCVRP_n{num_customers}_s{seed}",
        "num_customers": num_customers,
        "seed": seed,
        "vehicle_capacities": {
            "weight": vehicle_capacity_weight,
            "volume": vehicle_capacity_volume,
        },
        "fleet_size_upper_bound": max_vehicles,
        "total_demand": {
            "weight": round(total_w, 2),
            "volume": round(total_v, 2),
        },
        "nodes": nodes,
        "distances": distance_matrix,
    }

    return instance


def save_instance_to_file(instance: Dict[str, Any], filepath: str) -> None:
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(instance, f, indent=2)


if __name__ == "__main__":
    output_dir = os.path.join(os.path.dirname(__file__), "instances")
    test_sizes = [5, 10, 15, 20]
    
    print(f"Generating test instances in {output_dir}...")
    for size in test_sizes:
        inst = generate_instance(num_customers=size, seed=100 + size)
        filename = f"instance_n{size}.json"
        path = os.path.join(output_dir, filename)
        save_instance_to_file(inst, path)
        print(f" - Generated: {filename} ({size} customers)")
    print("Done!")

