use super::Cost;

/// A generic implementation of a Path
///
/// Stores a sequence of Nodes in ```path``` and the total Cost of traversing these Nodes in ```cost```.
/// Note that the individual costs of the steps within the Path cannot be retrieved through this struct.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Path<P> {
	/// the Path
	pub path: Vec<P>,
	/// the total Cost of the Path
	pub cost: Cost,
}

impl<P> Path<P> {
	/// creates a new Path with the given sequence of Nodes and total Cost
	/// ## Examples
	/// Basic usage:
	/// ```
	/// # use hierarchical_pathfinding::generics::Path;
	/// let path = Path::new(vec!['a', 'b', 'c'], 42);
	/// 
	/// assert_eq!(path.path, vec!['a', 'b', 'c']);
	/// assert_eq!(path.cost, 42);
	/// ```
	pub fn new(path: Vec<P>, cost: Cost) -> Path<P> {
		Path { path, cost }
	}

	/// appends a Node to the Path, adding it's Cost to the total Cost
	/// ## Examples
	/// Basic usage:
	/// ```
	/// # use hierarchical_pathfinding::generics::Path;
	/// let mut path = Path::new(vec!['a', 'b', 'c'], 42);
	/// path.append('d', 5);
	///
	/// assert_eq!(path.path, vec!['a', 'b', 'c', 'd']);
	/// assert_eq!(path.cost, 47);
	/// ```
	pub fn append(&mut self, node: P, cost: Cost) -> &mut Self {
		self.path.push(node);
		self.cost += cost;
		self
	}
}

use std::ops::{Deref, Index};

impl<P> Index<usize> for Path<P> {
	type Output = P;
	fn index(&self, index: usize) -> &P {
		&self.path[index]
	}
}

impl<P> Deref for Path<P> {
	type Target = [P];
	fn deref(&self) -> &[P] {
		&self.path
	}
}

use std::cmp::Ordering;

impl<P: Eq> Ord for Path<P> {
	fn cmp(&self, other: &Path<P>) -> Ordering {
		self.cost.cmp(&other.cost)
	}
}

impl<P: PartialEq> PartialOrd for Path<P> {
	fn partial_cmp(&self, other: &Path<P>) -> Option<Ordering> {
		Some(self.cost.cmp(&other.cost))
	}
}

use std::fmt;
impl<P: fmt::Display> fmt::Display for Path<P> {
	fn fmt(&self, fmt: &mut fmt::Formatter) -> fmt::Result {
		write!(fmt, "Path[Cost = {}]: ", self.cost)?;
		if self.path.is_empty() {
			write!(fmt, "<empty>")
		} else {
			write!(fmt, "{}", self.path[0])?;
			for p in self.path.iter().skip(1) {
				write!(fmt, " -> {}", p)?;
			}
			Ok(())
		}
	}
}

#[cfg(test)]
mod tests {

	use super::Path;
	#[test]
	fn path_index() {
		let path = Path::new(vec![4, 2, 0], 42);

		assert_eq!(path[0], 4);
		assert_eq!(path[1], 2);
		assert_eq!(path[2], 0);
	}

	#[test]
	fn path_display() {
		let path = Path::new(vec![4, 2, 0], 42);

		assert_eq!(&format!("{}", path), "Path[Cost = 42]: 4 -> 2 -> 0");
	}

	#[test]
	fn path_display_empty() {
		let path = Path::new(Vec::<i32>::new(), 0);

		assert_eq!(&format!("{}", path), "Path[Cost = 0]: <empty>");
	}
}
