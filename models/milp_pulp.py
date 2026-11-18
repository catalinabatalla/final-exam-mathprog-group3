"""
MILP Formulation for 2D-CVRP using PuLP.
Miller-Tucker-Zemlin (MTZ) formulation extended with 2D capacity tracking
(weight and volume constraints).
"""

import time
from typing import Dict, Any, Optional

try:
    import pulp
except ImportError:
    pulp = None


class PulpModel2DCVRP:
    def __init__(self, instance_data: Dict[str, Any]):
        self.instance = instance_data
        self.nodes = instance_data["nodes"]
        self.distances = instance_data["distances"]
        self.capacity_w = instance_data["vehicle_capacities"]["weight"]
        self.capacity_v = instance_data["vehicle_capacities"]["volume"]
        self.customers = [int(k) for k in self.nodes.keys() if int(k) != 0]
        self.all_nodes = [0] + self.customers
        self.num_vehicles = instance_data.get("fleet_size_upper_bound", len(self.customers))

    def solve(self, time_limit_sec: int = 60, msg: bool = False) -> Dict[str, Any]:
        if pulp is None:
            return {
                "status": "SolverUnavailable",
                "error": "PuLP package is not installed.",
                "total_cost": None,
                "runtime_sec": 0.0,
            }

        start_time = time.time()
        prob = pulp.LpProblem("2D_CVRP_MTZ", pulp.LpMinimize)

        # Vehicles set
        K = list(range(self.num_vehicles))

        # Decision variables:
        # x[i, j, k] = 1 if vehicle k travels from i to j
        x = {}
        for k in K:
            for i in self.all_nodes:
                for j in self.all_nodes:
                    if i != j:
                        x[i, j, k] = pulp.LpVariable(f"x_{i}_{j}_{k}", cat=pulp.LpBinary)

        # MTZ cumulative load variables per vehicle:
        # u_w[i, k] = cumulative weight after visiting i by vehicle k
        # u_v[i, k] = cumulative volume after visiting i by vehicle k
        u_w = {}
        u_v = {}
        for k in K:
            for i in self.customers:
                w_i = float(self.nodes[str(i)]["weight"])
                v_i = float(self.nodes[str(i)]["volume"])
                u_w[i, k] = pulp.LpVariable(f"u_w_{i}_{k}", lowBound=w_i, upBound=self.capacity_w, cat=pulp.LpContinuous)
                u_v[i, k] = pulp.LpVariable(f"u_v_{i}_{k}", lowBound=v_i, upBound=self.capacity_v, cat=pulp.LpContinuous)

        # Objective: minimize total distance
        prob += pulp.lpSum(
            float(self.distances[str(i)][str(j)]) * x[i, j, k]
            for k in K
            for i in self.all_nodes
            for j in self.all_nodes
            if i != j
        )

        # 1. Customer service: each customer visited exactly once
        for i in self.customers:
            prob += pulp.lpSum(x[i, j, k] for k in K for j in self.all_nodes if i != j) == 1

        # 2. Flow conservation for each node and vehicle
        for k in K:
            for i in self.all_nodes:
                prob += (
                    pulp.lpSum(x[j, i, k] for j in self.all_nodes if i != j)
                    - pulp.lpSum(x[i, j, k] for j in self.all_nodes if i != j)
                    == 0
                )

        # 3. Vehicle depot departure: at most one trip per vehicle
        for k in K:
            prob += pulp.lpSum(x[0, j, k] for j in self.customers) <= 1

        # 4. MTZ subtour elimination and 2D capacity propagation
        for k in K:
            for i in self.customers:
                for j in self.customers:
                    if i != j:
                        w_j = float(self.nodes[str(j)]["weight"])
                        v_j = float(self.nodes[str(j)]["volume"])

                        # Weight MTZ
                        prob += u_w[i, k] - u_w[j, k] + self.capacity_w * x[i, j, k] <= self.capacity_w - w_j

                        # Volume MTZ
                        prob += u_v[i, k] - u_v[j, k] + self.capacity_v * x[i, j, k] <= self.capacity_v - v_j

        # Solve
        solver = pulp.PULP_CBC_CMD(timeLimit=time_limit_sec, msg=msg)
        prob.solve(solver)
        runtime = time.time() - start_time

        status_str = pulp.LpStatus[prob.status]

        routes = []
        if prob.status == pulp.constants.LpStatusOptimal or prob.objective is not None:
            for k in K:
                # Reconstruct vehicle route
                curr = 0
                r = []
                while True:
                    next_node = None
                    for j in self.all_nodes:
                        if curr != j and pulp.value(x[curr, j, k]) and pulp.value(x[curr, j, k]) > 0.5:
                            next_node = j
                            break
                    if next_node is None or next_node == 0:
                        break
                    r.append(next_node)
                    curr = next_node
                if r:
                    routes.append(r)

            obj_val = round(pulp.value(prob.objective), 2) if prob.objective else None
            return {
                "status": status_str,
                "total_cost": obj_val,
                "num_vehicles": len(routes),
                "routes": routes,
                "runtime_sec": round(runtime, 3),
            }

        return {
            "status": status_str,
            "total_cost": None,
            "runtime_sec": round(runtime, 3),
        }
