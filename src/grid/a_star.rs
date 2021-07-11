use super::{HeuristicElement, Path};
use crate::{neighbors::Neighborhood, Point, PointMap};

use std::cmp::Ordering;
use std::collections::BinaryHeap;

pub fn a_star_search<N: Neighborhood>(
    neighborhood: &N,
    mut valid: impl FnMut(Point) -> bool,
    mut get_cost: impl FnMut(Point) -> isize,
    start: Point,
    goal: Point,
    size_hint: usize,
) -> Option<Path<Point>> {
    if get_cost(start) < 0 {
        return None;
    }
    if start == goal {
        return Some(Path::from_slice(&[start, start], 0));
    }
    let mut visited = PointMap::with_capacity(size_hint);
    let mut next = BinaryHeap::with_capacity(size_hint / 2);
    next.push(HeuristicElement(start, 0, 0));
    visited.insert(start, (0, start));

    let mut all_neighbors = vec![];

    while let Some(HeuristicElement(current_id, current_cost, _)) = next.pop() {
        if current_id == goal {
            break;
        }
        match current_cost.cmp(&visited[&current_id].0) {
            Ordering::Greater => continue,
            Ordering::Equal => {}
            Ordering::Less => panic!("Binary Heap failed"),
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
            if get_cost(other_id) < 0 && other_id != goal {
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
                let heuristic = neighborhood.heuristic(other_id, goal);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unreachable_goal() {
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
        let goal = (2, 0);

        let path = a_star_search(&neighborhood, |_| true, cost_fn(&grid), start, goal, 40);

        assert!(path.is_none());
    }

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
        let goal = (4, 4);
        let path = a_star_search(&neighborhood, |_| true, cost_fn(&grid), start, goal, 40);

        assert!(path.is_some());
        let path = path.unwrap();

        assert_eq!(path.cost(), 12);
    }
}
