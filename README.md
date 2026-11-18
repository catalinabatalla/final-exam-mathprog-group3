# Mathematical Programming - Final Exam (Group 3)

## Problem Description
A logistic company has to organize the short-haul distribution of goods from its central depot to a set of customers in the same city:
- Directed graph with a depot node (0) and customer nodes ($1 \dots n$).
- Two directed arcs between each pair of nodes, with distances $c_{ij}$ that can be asymmetric due to one-way streets.
- Unlimited fleet of homogeneous vehicles stationed at the depot.
- Vehicles have two capacity limits:
  - Maximum Weight capacity ($Q_W$).
  - Maximum Volume capacity ($Q_V$).
- Each customer requires:
  - Weight demand ($w_i$).
  - Volume demand ($v_i$).
  - Weight and volume are not directly proportional.
- Each customer must be served by exactly one vehicle.
- Objective: Minimize the total distance traveled by all vehicles.

## Current Step: Step 1 - Problem Setup & Data Structure
- `data/instance_generator.py`: Simple script to define and generate test instances.
- Next steps will be done step-by-step:
  1. Define and understand the instance format.
  2. Formulate the baseline mathematical model (MILP).
  3. Implement the baseline solver.
  4. Test on a tiny example.
