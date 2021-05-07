use super::*;

use std::cmp::Ordering;
use std::collections::BinaryHeap;

pub fn dijkstra_search(
    nodes: &NodeMap,
    start: NodeID,
    goals: &[NodeID],
    only_closest_goal: bool,
) -> NodeIDMap<Path<NodeID>> {
    let mut visited = NodeIDMap::default();
    let mut next = BinaryHeap::new();
    next.push(Element(start, 0));
    visited.insert(start, (0, start));

    let mut remaining_goals: NodeIDSet = goals.iter().copied().collect();

    let mut goal_costs = NodeIDMap::with_capacity_and_hasher(goals.len(), Default::default());

    while let Some(Element(current_id, current_cost)) = next.pop() {
        match current_cost.cmp(&visited[&current_id].0) {
            Ordering::Greater => continue,
            Ordering::Equal => {}
            Ordering::Less => panic!("Binary Heap failed"),
        }

        if remaining_goals.remove(&current_id) {
            goal_costs.insert(current_id, current_cost);
            if only_closest_goal || remaining_goals.is_empty() {
                break;
            }
        }

        let current = &nodes[current_id];

        for (&other_id, path) in current.edges.iter() {
            let other_cost = current_cost + path.cost();

            let mut needs_visit = true;
            if let Some((prev_cost, prev_id)) = visited.get_mut(&other_id) {
                if *prev_cost > other_cost {
                    *prev_cost = other_cost;
                    *prev_id = current_id;
                } else {
                    needs_visit = false;
                }
            } else {
                visited.insert(other_id, (other_cost, current_id));
            }

            if needs_visit {
                next.push(Element(other_id, other_cost));
            }
        }
    }

    let mut goal_data = NodeIDMap::with_capacity_and_hasher(goal_costs.len(), Default::default());

    for (&goal, &cost) in goal_costs.iter() {
        let steps = {
            let mut steps = vec![];
            let mut current = goal;

            while current != start {
                steps.push(current);
                let (_, prev) = visited[&current];
                current = prev;
            }
            steps.push(start);
            steps.reverse();
            steps
        };
        goal_data.insert(goal, Path::new(steps, cost));
    }

    goal_data
}
