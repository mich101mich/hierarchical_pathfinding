use super::*;
use crate::neighbors::Neighborhood;

use std::cmp::Ordering;
use std::collections::BinaryHeap;

pub(crate) fn a_star_search<N: Neighborhood>(
    nodes: &NodeList,
    start: NodeID,
    goal: NodeID,
    neighborhood: &N,
    size_hint: usize,
) -> Option<Path<NodeID>> {
    if start == goal {
        return Some(Path::from_slice(&[start, start], 0));
    }
    let mut visited = NodeIDMap::with_capacity(size_hint);
    let mut next = BinaryHeap::with_capacity(size_hint / 2);
    next.push(HeuristicElement(start, 0, 0));
    visited.insert(start, (0, start));

    while let Some(HeuristicElement(current_id, current_cost, _)) = next.pop() {
        if current_id == goal {
            break;
        }
        match current_cost.cmp(&visited[&current_id].0) {
            Ordering::Greater => continue,
            Ordering::Equal => {}
            Ordering::Less => panic!("Binary Heap failed"),
        }

        let current = &nodes[current_id];

        for (&other_id, path) in current.edges.iter() {
            let other_cost = current_cost + path.cost();
            let other = &nodes[other_id];

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
                let heuristic = neighborhood.heuristic(current.pos, other.pos);
                next.push(HeuristicElement(
                    other_id,
                    other_cost,
                    other_cost + heuristic,
                ));
            }
        }
    }

    if !visited.contains_key(&goal) {
        return None;
    }

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

    Some(Path::new(steps, visited[&goal].0))
}
