use super::super::{ordered_insert, Path};
use crate::{Point, PointMap};

/// Searches a Graph using [Dijkstra's Algorithm](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm).
///
/// The Generic type Parameter `Point` is supposed to uniquely identify a Node in the Graph.
/// This may be a Number, String, a Grid position, ... as long as it can be compared, hashed and copied.
/// Note that it is advised to choose a short representation for the Point, since it will be copied several times.
///
/// This function can be used to search for several Goals and will try to calculate a Path for every provided Goal.
/// It stops as soon as it has the shortest Path to every Goal, or when all reachable Nodes have been expanded.
///
/// ## Examples
/// Basic usage:
/// ```
/// # use hierarchical_pathfinding::generics::grid::dijkstra_search;
/// # use hierarchical_pathfinding::{prelude::*, Point};
/// // create and initialize Grid
/// // 0 = empty, 1 = swamp, 2 = wall
/// let mut grid = [
///     [0, 2, 0, 0, 0],
///     [0, 2, 2, 2, 2],
///     [0, 1, 0, 0, 0],
///     [0, 1, 0, 2, 0],
///     [0, 0, 0, 2, 0],
/// ];
/// let (width, height) = (grid.len(), grid[0].len());
///
/// let neighborhood = ManhattanNeighborhood::new(width, height);
///
/// const COST_MAP: [isize; 3] = [1, 10, -1];
///
/// fn cost_fn<'a>(grid: &'a [[usize; 5]; 5]) -> impl 'a + FnMut(Point) -> isize {
///     move |(x, y)| COST_MAP[grid[y][x]]
/// }
///
/// let start = (0, 0);
/// let goals = [(4, 4), (2, 0)];
///
/// let paths = dijkstra_search(
///     |point| neighborhood.get_all_neighbors(point),
///     cost_fn(&grid),
///     start,
///     &goals,
/// );
///
/// // (4, 4) is reachable
/// assert!(paths.contains_key(&goals[0]));
///
/// // (2, 0) is not reachable
/// assert!(!paths.contains_key(&goals[1]));
/// ```
///
/// ## Solid Goals
/// It is possible to calculate the shortest Path to for example a Wall and other non-walkable
/// Nodes using this function. To do that, simply supply a Function to the `get_cost` Parameter
/// that returns a negative number for Nodes that should not be used as part of an actual Path.
/// In the case that a Path to a non-walkable Goal is requested, the neighbor of that Goal with the
/// shortest Path from the Start is returned, if any is reachable. "neighbor" in this context is
/// a Node for which `get_all_neighbors` contains the Goal.
///
/// ## Arguments
/// - `get_all_neighbors` - a Function that takes a Node and returns all other Nodes reachable from that Node.
///     The returned value is the `Point` of the neighbor.
/// - `get_cost` - a Function that takes a Node and returns the Cost required to walk across that Node.
///     Negative values indicate Nodes that cannot be walked across.
/// - `start` - the starting Node
/// - `goals` - the Goals that this function is supposed to search for
///
/// ## Returns
/// a HashMap with all reachable Goal's Points as the Key and the shortest Path to reach that Goal as Value.
/// The first Node in the Path is always the `start` and the last is the corresponding Goal
pub fn dijkstra_search<NeighborIter: Iterator<Item = Point>>(
	mut get_all_neighbors: impl FnMut(Point) -> NeighborIter,
	mut get_cost: impl FnMut(Point) -> isize,
	start: Point,
	goals: &[Point],
) -> PointMap<Path<Point>> {
	let mut visited = PointMap::default();
	let mut next = vec![(start, 0)];
	visited.insert(start, (0, start));

	let mut remaining_goals = goals.to_vec();

	let mut goal_costs = PointMap::with_capacity_and_hasher(goals.len(), Default::default());

	while let Some((current_id, _)) = next.pop() {
		let cost = visited[&current_id].0;

		let mut found_one = false;
		for &goal_id in remaining_goals.iter() {
			if current_id == goal_id {
				goal_costs.insert(goal_id, cost);
				found_one = true;
			}
		}
		if found_one {
			remaining_goals.retain(|&id| id != current_id);
			if remaining_goals.is_empty() {
				break;
			}
		}

		let delta_cost = get_cost(current_id);
		if delta_cost < 0 {
			continue;
		}
		let delta_cost = delta_cost as usize;

		for other_id in get_all_neighbors(current_id) {
			let other_cost = cost + delta_cost;

			if get_cost(other_id) < 0 {
				let mut is_goal = false;
				for &goal_id in remaining_goals.iter() {
					if other_id == goal_id {
						is_goal = true;
					}
				}
				if !is_goal {
					continue;
				}
			}

			if let Some(&(prev_cost, _)) = visited.get(&other_id) {
				if prev_cost > other_cost {
					next.retain(|&(id, _)| id != other_id);
				}
			}

			if !visited.contains_key(&other_id) || visited[&other_id].0 > other_cost {
				ordered_insert(&mut next, (other_id, other_cost), |&(_, cost)| cost);
				visited.insert(other_id, (other_cost, current_id));
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
