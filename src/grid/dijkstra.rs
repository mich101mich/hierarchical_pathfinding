use super::{Element, Path};
use crate::{neighbors::Neighborhood, Point, PointMap, PointSet};

use std::cmp::Ordering;
use std::collections::BinaryHeap;

pub fn dijkstra_search<N: Neighborhood>(
    neighborhood: &N,
    mut valid: impl FnMut(Point) -> bool,
    mut get_cost: impl FnMut(Point) -> isize,
    start: Point,
    goals: &[Point],
    only_closest_goal: bool,
) -> PointMap<Path<Point>> {
    let mut visited = PointMap::default();
    let mut next = BinaryHeap::new();
    next.push(Element(start, 0));
    visited.insert(start, (0, start));

    let mut remaining_goals: PointSet = goals.iter().copied().collect();

    let mut goal_costs = PointMap::with_capacity_and_hasher(goals.len(), Default::default());

    let mut all_neighbors = vec![];

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

        let delta_cost = get_cost(current_id);
        if delta_cost < 0 {
            continue;
        }
        let other_cost = current_cost + delta_cost as usize;

        all_neighbors.clear();
        neighborhood.get_all_neighbors(current_id, &mut all_neighbors);
        for &other_id in all_neighbors.iter() {
            if !valid(other_id) {
                continue;
            }
            if get_cost(other_id) < 0 && !remaining_goals.contains(&other_id) {
                continue;
            }

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

    let mut goal_data = PointMap::with_capacity_and_hasher(goal_costs.len(), Default::default());

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic() {
        use crate::prelude::*;

        // create and initialize Grid
        // 0 = empty, 1 = swamp, 2 = wall
        let grid = [
            [0, 2, 0, 0, 0],
            [0, 2, 2, 2, 2],
            [0, 1, 0, 0, 0],
            [0, 1, 0, 2, 0],
            [0, 0, 0, 2, 0],
        ];
        let (width, height) = (grid.len(), grid[0].len());

        let neighborhood = ManhattanNeighborhood::new(width, height);

        const COST_MAP: [isize; 3] = [1, 10, -1];

        fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + FnMut(Point) -> isize {
            move |(x, y)| COST_MAP[grid[y][x]]
        }

        let start = (0, 0);
        let goals = [(4, 4), (2, 0)];

        let paths = dijkstra_search(
            &neighborhood,
            |_| true,
            cost_fn(&grid),
            start,
            &goals,
            false,
        );

        // (4, 4) is reachable
        assert!(paths.contains_key(&goals[0]));

        // (2, 0) is not reachable
        assert!(!paths.contains_key(&goals[1]));
    }
}
