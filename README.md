# Vehicle Routing Problem with Two-Dimensional Capacity (2D-CVRP)
### Mathematical Programming (Course 97271) - Final Exam Project (Group 3)

---

## 1. Problem Overview

This project addresses the short-haul distribution problem of goods from a central depot to a set of customers in the same city.

### Characteristics & Specifications:
- **Graph Topology:** Directed graph $G = (V, A)$ where:
  - $V = \{0\} \cup \{1, \dots, n\}$ ($0$ represents the central depot, and $1, \dots, n$ represent customer locations).
  - Directed arcs $(i, j) \in A$ for each pair of nodes in both directions.
  - Distances/costs $c_{ij} \ge 0$ may be asymmetric ($c_{ij} \ne c_{ji}$) due to one-way streets and urban topology.
- **Fleet:** Unlimited supply of homogeneous vehicles stationed at the depot.
- **Two-Dimensional Capacity Constraints:** Each vehicle has:
  - Maximum weight capacity $Q_W$.
  - Maximum volume capacity $Q_V$.
  - Goods are flexible enough to always fit physically in the vehicle container provided total weight and total volume do not exceed the respective capacities.
- **Customer Demand:** Each customer $i \in \{1, \dots, n\}$ requires:
  - Weight demand $w_i > 0$.
  - Volume demand $v_i > 0$.
  - Weight and volume demands are **not directly proportional**.
  - Each customer must be serviced by **exactly one vehicle** (no split delivery).
- **Objective:** Serve all customers at minimal total operational cost (total distance traveled by all vehicles).

---

## 2. Mathematical Formulation (MILP)

We model this problem as a 2-Dimensional Capacitated Vehicle Routing Problem (2D-CVRP) with MTZ (Miller-Tucker-Zemlin) formulation for subtour elimination extended with multi-dimensional capacity tracking.

### Sets and Indices:
- $V = \{0, 1, \dots, n\}$: Set of all vertices ($0$ is depot, $V_C = \{1, \dots, n\}$ are customers).
- $A = \{(i, j) : i, j \in V, i \ne j\}$: Set of directed arcs.
- $K = \{1, \dots, K_{\max}\}$: Set of available vehicles stationed at the depot.

### Parameters:
- $c_{ij}$: Travel distance/cost from node $i$ to node $j$.
- $w_i, v_i$: Weight and volume demands of customer $i$ ($w_0 = 0, v_0 = 0$).
- $Q_W, Q_V$: Maximum weight and volume capacity per vehicle.

### Decision Variables:
- $x_{ijk} \in \{0, 1\}$: Binary variable indicating if vehicle $k$ traverses directed arc $(i, j)$.
- $u_{ik}^W \ge 0$: Cumulative weight collected/delivered up to node $i$ by vehicle $k$.
- $u_{ik}^V \ge 0$: Cumulative volume collected/delivered up to node $i$ by vehicle $k$.

### Objective Function:
$$\min \sum_{k \in K} \sum_{(i,j) \in A} c_{ij} x_{ijk}$$

### Constraints:
1. **Customer Service:**
   $$\sum_{k \in K} \sum_{j \in V, j \ne i} x_{ijk} = 1 \quad \forall i \in V_C$$

2. **Flow Conservation:**
   $$\sum_{j \in V, j \ne i} x_{ijk} - \sum_{j \in V, j \ne i} x_{jik} = 0 \quad \forall i \in V, \forall k \in K$$

3. **Depot Departure:**
   $$\sum_{j \in V_C} x_{0jk} \le 1 \quad \forall k \in K$$

4. **Two-Dimensional Capacity Bounds & Subtour Elimination (MTZ):**
   $$u_{ik}^W - u_{jk}^W + Q_W x_{ijk} \le Q_W - w_j \quad \forall i, j \in V_C, i \ne j, \forall k \in K$$
   $$u_{ik}^V - u_{jk}^V + Q_V x_{ijk} \le Q_V - v_j \quad \forall i, j \in V_C, i \ne j, \forall k \in K$$
   $$w_i \le u_{ik}^W \le Q_W \quad \forall i \in V_C, \forall k \in K$$
   $$v_i \le u_{ik}^V \le Q_V \quad \forall i \in V_C, \forall k \in K$$

---

## 3. Project Structure

```text
├── data/
│   ├── instances/            # JSON/CSV instance files
│   └── instance_generator.py # Synthetic benchmark generator with asymmetric distances
├── models/
│   ├── milp_pulp.py          # Baseline MILP formulation using PuLP / CBC
│   └── heuristics.py         # Baseline heuristic (Savings algorithm / Nearest Neighbor + 2-opt)
├── results/                  # Computational logs, outputs, and CSV reports
├── tests/                    # Unit and regression tests
├── run_experiments.py        # Benchmark driver script comparing baseline vs heuristics
├── requirements.txt          # Python dependencies
└── README.md
```

---

## 4. Setup and Execution

### 1. Requirements Installation
```bash
pip install -r requirements.txt
```

### 2. Generate Benchmark Instances
```bash
python data/instance_generator.py
```

### 3. Run Experiments & Comparisons
```bash
python run_experiments.py
```
