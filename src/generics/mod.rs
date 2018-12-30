//! A Module for generic implementations.
//!
//! This Module is primarily used as the internal Library Backend for the actual implementations in the other Modules.
//! You may use the provided functionality for other purposes,
//! but it is recommended to use implementations from other Crates instead.

mod path;
pub use self::path::Path;

mod dijkstra;
pub use self::dijkstra::dijkstra_search;

mod a_star;
pub use self::a_star::a_star_search;

/// a Type to represent the Cost of traversing a Node
pub type Cost = usize;

fn ordered_insert<T, V, F>(vector: &mut Vec<T>, element: T, get_value: F)
where
	V: Ord,
	F: Fn(&T) -> V,
{
	let value = get_value(&element);
	for i in 0..vector.len() {
		if get_value(&vector[i]) <= value {
			vector.insert(i, element);
			return;
		}
	}
	vector.push(element);
}
