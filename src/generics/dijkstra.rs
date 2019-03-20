use super::{ordered_insert, Cost, Path};
use std::collections::HashMap;
use std::hash::Hash;

/// Searches a Graph using [Dijkstra's Algorithm](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm).
///
/// The Generic type Parameter `Id` is supposed to uniquely identify a Node in the Graph.
/// This may be a Number, String, a Grid position, ... as long as it can be compared, hashed and copied.
/// Note that it is advised to choose a short representation for the Id, since it will be copied several times.
///
/// This function can be used to search for several Goals and will try to calculate a Path for every provided Goal.
/// It stops as soon as it has the shortest Path to every Goal, or when all reachable Nodes have been expanded.
///
/// ## Examples
/// Basic usage:
/// ```
/// # use hierarchical_pathfinding::generics::dijkstra_search;
/// // A     B--2--E
/// // |\     
/// // | \    
/// // 1  9   
/// // |   \  
/// // |    \
/// // C--6--D
/// let (A, B, C, D, E) = (0, 1, 2, 3, 4);
/// let cost_matrix: [[i32; 5]; 5] = [
/// //    A,  B,  C,  D,  E
/// 	[-1, -1,  1,  9, -1], // A
/// 	[-1, -1, -1, -1,  2], // B
/// 	[ 1, -1, -1,  6, -1], // C
/// 	[ 9, -1,  6, -1, -1], // D
/// 	[-1,  2, -1, -1, -1], // E
/// ];
///
/// let result = dijkstra_search(
/// 	|point| { // get_all_neighbors
/// 		cost_matrix[point]
/// 			.iter()
/// 			.enumerate()
/// 			.filter(|&(_, cost)| *cost != -1)
/// 			.map(|(id, cost)| (id, *cost as usize))
/// 	},
/// 	|_| true, // is_walkable
/// 	A, // start
/// 	&[D, E], // goals
/// );
///
/// // if the Goal is reachable, the Path is added to the Map
/// assert!(result.contains_key(&D));
/// let path = &result[&D];
/// assert_eq!(path.path, vec![A, C, D]);
/// assert_eq!(path.cost, 7);
///
/// // if the Goal is not reachable, there won't be an entry in the Map
/// assert!(!result.contains_key(&E));
/// ```
///
/// ## Solid Goals
/// It is possible to calculate the shortest Path to for example a Wall and other non-walkable
/// Nodes using this function. To do that, simply supply a Function to the `is_walkable` Parameter
/// that returns `false` for Nodes that should not be used as part of an actual Path. If there are
/// no such Nodes in the Graph, `is_walkable` may simply be set to `|_| true`.  
/// In the case that a Path to a non-walkable Goal is requested, the neighbor of that Goal with the
/// shortest Path from the Start is returned, if any is reachable. "neighbor" in this context is
/// a Node for which `get_all_neighbors` contains the Goal.
///
/// ## Arguments
/// - `get_all_neighbors` - a Function that takes a Node and returns all other Nodes reachable from that Node.
/// 	The returned value is a Tuple of the `Id` of the neighbor and the Cost to get there.
/// - `start` - the starting Node
/// - `goals` - the Goals that this function is supposed to search for
///
/// ## Returns
/// a HashMap with all reachable Goal's Ids as the Key and the shortest Path to reach that Goal as Value.
/// The first Node in the Path is always the `start` and the last is the corresponding Goal
pub fn dijkstra_search<Id: Copy + Eq + Hash, NeighborIter: Iterator<Item = (Id, Cost)>>(
	get_all_neighbors: impl Fn(Id) -> NeighborIter,
	is_walkable: impl Fn(Id) -> bool,
	start: Id,
	goals: &[Id],
) -> HashMap<Id, Path<Id>> {
	let mut visited = ::std::collections::HashMap::new();
	let mut next = vec![(start, 0)];
	visited.insert(start, (0, start));

	let mut remaining_goals = goals.to_vec();

	let mut goal_costs = HashMap::with_capacity(goals.len());

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

		for (other_id, delta_cost) in get_all_neighbors(current_id) {
			let other_cost = cost + delta_cost;

			if !is_walkable(other_id) {
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

	let mut goal_data = HashMap::with_capacity(goal_costs.len());

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
