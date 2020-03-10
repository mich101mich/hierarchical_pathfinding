use super::super::{ordered_insert, Cost, Path};
use crate::{node_id::*, NodeID};

/// Searches a Graph using the [A* Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) in a Node Graph with [`NodeID`]s.
///
/// ## Arguments
/// - `get_all_neighbors` - a Function that takes a Node and returns all other Nodes reachable from that Node.
///     The returned value is a Tuple of the `NodeID` of the neighbor and the Cost to get there.
/// - `is_walkable` - a Function that determines if a Node can be walked over. see [Solid Goals](../grid/fn.a_star_search.html#solid-goals) for more info
/// - `start` - the starting Node
/// - `goal` - the Goal that this function is supposed to search for
/// - `heuristic` - the Heuristic Function of the A* Algorithm
///
/// ## Returns
/// the Path, if one was found, or None if the `goal` is unreachable.
/// The first Node in the Path is always the `start` and the last is the `goal`
pub fn a_star_search<NeighborIter: Iterator<Item = (NodeID, Cost)>>(
	mut get_all_neighbors: impl FnMut(NodeID) -> NeighborIter,
	mut is_walkable: impl FnMut(NodeID) -> bool,
	start: NodeID,
	goal: NodeID,
	mut heuristic: impl FnMut(NodeID) -> Cost,
) -> Option<Path<NodeID>> {
	if start == goal {
		return Some(Path::new(vec![start, start], 0));
	}
	let mut visited = node_id_map();
	let mut next = vec![(start, 0)];
	visited.insert(start, (0, start));

	'search: while let Some((current_id, _)) = next.pop() {
		if current_id == goal {
			break 'search;
		}
		let current_cost = visited[&current_id].0;

		for (other_id, delta_cost) in get_all_neighbors(current_id) {
			let other_cost = current_cost + delta_cost;

			if !is_walkable(other_id) && other_id != goal {
				continue;
			}

			let heuristic = heuristic(other_id);

			if let Some(&(prev_cost, _)) = visited.get(&other_id) {
				if prev_cost > other_cost {
					next.retain(|&(id, _)| id != other_id);
				}
			}

			if !visited.contains_key(&other_id) || visited[&other_id].0 > other_cost {
				ordered_insert(
					&mut next,
					(other_id, other_cost + heuristic),
					|&(_, cost)| cost,
				);
				visited.insert(other_id, (other_cost, current_id));
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
