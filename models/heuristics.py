"""
Heuristic Algorithms for the 2D-CVRP:
1. Clarke & Wright Savings Algorithm (adapted for 2D capacity & asymmetric distance matrix)
2. Nearest Neighbor Heuristic with 2-opt local search post-optimization
"""

from typing import List, Dict, Any, Tuple


class Heuristic2DCVRP:
    def __init__(self, instance_data: Dict[str, Any]):
        self.instance = instance_data
        self.nodes = instance_data["nodes"]
        self.distances = instance_data["distances"]
        self.capacity_w = instance_data["vehicle_capacities"]["weight"]
        self.capacity_v = instance_data["vehicle_capacities"]["volume"]
        self.customers = [int(k) for k in self.nodes.keys() if int(k) != 0]

    def get_dist(self, i: int, j: int) -> float:
        return self.distances[str(i)][str(j)]

    def get_demand(self, node_id: int) -> Tuple[float, float]:
        node = self.nodes[str(node_id)]
        return float(node["weight"]), float(node["volume"])

    def route_cost(self, route: List[int]) -> float:
        if not route:
            return 0.0
        full_path = [0] + route + [0]
        cost = 0.0
        for idx in range(len(full_path) - 1):
            cost += self.get_dist(full_path[idx], full_path[idx + 1])
        return cost

    def route_demands(self, route: List[int]) -> Tuple[float, float]:
        tw, tv = 0.0, 0.0
        for c in route:
            w, v = self.get_demand(c)
            tw += w
            tv += v
        return tw, tv

    def is_feasible(self, route: List[int]) -> bool:
        tw, tv = self.route_demands(route)
        return (tw <= self.capacity_w + 1e-6) and (tv <= self.capacity_v + 1e-6)

    def nearest_neighbor(self) -> Dict[str, Any]:
        """
        Greedy Nearest Neighbor heuristic constructing routes one vehicle at a time.
        """
        unvisited = set(self.customers)
        routes: List[List[int]] = []

        while unvisited:
            current_route: List[int] = []
            curr_w, curr_v = 0.0, 0.0
            curr_loc = 0

            while unvisited:
                # Find feasible unvisited customer with minimal distance from curr_loc
                best_next = None
                best_dist = float("inf")

                for cand in unvisited:
                    w, v = self.get_demand(cand)
                    if (curr_w + w <= self.capacity_w + 1e-6) and (curr_v + v <= self.capacity_v + 1e-6):
                        d = self.get_dist(curr_loc, cand)
                        if d < best_dist:
                            best_dist = d
                            best_next = cand

                if best_next is not None:
                    current_route.append(best_next)
                    w, v = self.get_demand(best_next)
                    curr_w += w
                    curr_v += v
                    curr_loc = best_next
                    unvisited.remove(best_next)
                else:
                    # Vehicle is full or cannot reach any remaining customer within capacity
                    break

            routes.append(current_route)

        total_cost = sum(self.route_cost(r) for r in routes)
        return {
            "algorithm": "Nearest Neighbor",
            "routes": routes,
            "num_vehicles": len(routes),
            "total_cost": round(total_cost, 2),
        }

    def two_opt_route(self, route: List[int]) -> List[int]:
        """
        2-opt intra-route improvement respecting asymmetric arc costs.
        """
        if len(route) < 3:
            return route

        improved = True
        best_route = list(route)
        best_cost = self.route_cost(best_route)

        while improved:
            improved = False
            for i in range(len(best_route) - 1):
                for j in range(i + 1, len(best_route)):
                    # Reverse segment from i to j
                    new_route = best_route[:i] + best_route[i : j + 1][::-1] + best_route[j + 1 :]
                    new_cost = self.route_cost(new_route)
                    if new_cost < best_cost - 1e-6:
                        best_route = new_route
                        best_cost = new_cost
                        improved = True
                        break
                if improved:
                    break

        return best_route

    def nearest_neighbor_2opt(self) -> Dict[str, Any]:
        """
        Runs Nearest Neighbor followed by 2-opt refinement on each route.
        """
        nn_res = self.nearest_neighbor()
        optimized_routes = [self.two_opt_route(r) for r in nn_res["routes"]]
        total_cost = sum(self.route_cost(r) for r in optimized_routes)
        return {
            "algorithm": "Nearest Neighbor + 2-opt",
            "routes": optimized_routes,
            "num_vehicles": len(optimized_routes),
            "total_cost": round(total_cost, 2),
        }

    def clarke_wright_savings(self) -> Dict[str, Any]:
        """
        Clarke & Wright savings algorithm adapted for asymmetric costs and 2D capacity.
        Savings s_ij = c_{i0} + c_{0j} - c_{ij}
        """
        # Initially, each customer has their own dedicated route [i]
        routes: List[List[int]] = [[c] for c in self.customers]

        # Calculate savings for all pairs (i, j) with i != j
        savings: List[Tuple[float, int, int]] = []
        for i in self.customers:
            for j in self.customers:
                if i != j:
                    s_ij = self.get_dist(i, 0) + self.get_dist(0, j) - self.get_dist(i, j)
                    savings.append((s_ij, i, j))

        savings.sort(key=lambda x: x[0], reverse=True)

        for s_ij, i, j in savings:
            # Find routes containing i and j
            route_i = None
            route_j = None
            for r in routes:
                if i in r:
                    route_i = r
                if j in r:
                    route_j = r

            # Check if they can be merged:
            # i must be the end of route_i, and j must be the start of route_j, and route_i != route_j
            if route_i is not None and route_j is not None and route_i != route_j:
                if route_i[-1] == i and route_j[0] == j:
                    merged = route_i + route_j
                    if self.is_feasible(merged):
                        routes.remove(route_i)
                        routes.remove(route_j)
                        routes.append(merged)

        # Apply 2-opt to finalize merged routes
        optimized_routes = [self.two_opt_route(r) for r in routes]
        total_cost = sum(self.route_cost(r) for r in optimized_routes)

        return {
            "algorithm": "Clarke-Wright Savings + 2-opt",
            "routes": optimized_routes,
            "num_vehicles": len(optimized_routes),
            "total_cost": round(total_cost, 2),
        }
