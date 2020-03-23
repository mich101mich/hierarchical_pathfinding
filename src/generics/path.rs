use super::Cost;
use std::rc::Rc;

/// A generic implementation of a Path
///
/// Stores a sequence of Nodes and the total Cost of traversing these Nodes.
/// Note that the individual costs of the steps within the Path cannot be retrieved through this
/// struct.
///
/// This struct does not own the actual Path, it merely keeps an [`Rc`] to it. This makes cloning
/// and reversing very efficient, but makes them immutable and limits some ways to access the
/// contents
#[derive(Debug, Clone, PartialEq, Eq)]
#[allow(missing_doc_code_examples)]
pub struct Path<P> {
	path: Rc<[P]>,
	cost: Cost,
	is_reversed: bool,
}

impl<P> Path<P> {
	/// creates a new Path with the given sequence of Nodes and total Cost
	/// ## Examples
	/// Basic usage:
	/// ```
	/// # use hierarchical_pathfinding::generics::Path;
	/// let path = Path::new(vec!['a', 'b', 'c'], 42);
	///
	/// assert_eq!(path, vec!['a', 'b', 'c']);
	/// assert_eq!(path.cost(), 42);
	/// ```
	pub fn new(path: Vec<P>, cost: Cost) -> Path<P> {
		Path {
			path: path.into(),
			cost,
			is_reversed: false,
		}
	}

	/// creates a new Path with the given sequence of Nodes and total Cost
	/// ## Examples
	/// Basic usage:
	/// ```
	/// # use hierarchical_pathfinding::generics::Path;
	/// let path = Path::from_slice(&['a', 'b', 'c'], 42);
	///
	/// assert_eq!(path, vec!['a', 'b', 'c']);
	/// assert_eq!(path.cost(), 42);
	/// ```
	pub fn from_slice(path: &[P], cost: Cost) -> Path<P>
	where
		P: Clone,
	{
		Path {
			path: path.into(),
			cost,
			is_reversed: false,
		}
	}

	/// Returns the Cost of the Path
	pub fn cost(&self) -> Cost {
		self.cost
	}

	/// Returns the length of the Path
	pub fn len(&self) -> usize {
		self.path.len()
	}

	/// Returns if the Path is empty
	pub fn is_empty(&self) -> bool {
		self.path.is_empty()
	}

	/// Returns a reversed version of the Path.
	///
	/// `start_cost` is what need to be subtracted, and `end_cost` is what needs to be
	/// added to the cost in the case of asymmetric paths. Can be set to 0 for symmetric paths.
	///
	/// This operation is low cost since Paths are based on [`Rc`]s.
	///
	/// ## Examples
	/// Basic usage:
	/// ```
	/// # use hierarchical_pathfinding::generics::Path;
	/// let path = Path::new(vec!['a', 'b', 'c'], 42);
	/// let reversed = path.reversed(5, 2);
	///
	/// assert_eq!(reversed, vec!['c', 'b', 'a']);
	/// assert_eq!(reversed.cost(), 39);
	/// ```
	pub fn reversed(&self, start_cost: Cost, end_cost: Cost) -> Path<P>
	where
		P: Clone,
	{
		Path {
			path: self.path.clone(),
			cost: self.cost - start_cost + end_cost,
			is_reversed: !self.is_reversed,
		}
	}

	/// Returns an Iterator over the Path
	pub fn iter(&self) -> Iter<P> {
		Iter {
			iter: self.path.iter(),
			reversed: self.is_reversed,
		}
	}

	/// Extracts a Vec from the Path, cloning the data
	pub fn as_vec(&self) -> Vec<P>
	where
		P: Clone,
	{
		self.iter().cloned().collect()
	}
}

use std::ops::Index;

impl<P> Index<usize> for Path<P> {
	type Output = P;
	fn index(&self, index: usize) -> &P {
		let index = if self.is_reversed {
			self.path.len() - index - 1
		} else {
			index
		};
		&self.path[index]
	}
}

#[derive(Debug)]
pub struct Iter<'a, P> {
	iter: std::slice::Iter<'a, P>,
	reversed: bool,
}

impl<'a, P> Iterator for Iter<'a, P> {
	type Item = &'a P;
	fn next(&mut self) -> Option<Self::Item> {
		if self.reversed {
			self.iter.next_back()
		} else {
			self.iter.next()
		}
	}
	fn size_hint(&self) -> (usize, Option<usize>) {
		self.iter.size_hint()
	}
}

impl<P> DoubleEndedIterator for Iter<'_, P> {
	fn next_back(&mut self) -> Option<Self::Item> {
		if self.reversed {
			self.iter.next()
		} else {
			self.iter.next_back()
		}
	}
}
impl<P> ExactSizeIterator for Iter<'_, P> {}
impl<P> std::iter::FusedIterator for Iter<'_, P> {}

impl<P: PartialEq> PartialEq<Vec<P>> for Path<P> {
	fn eq(&self, rhs: &Vec<P>) -> bool {
		// we can't just use slice's eq because self might be reversed
		self.len() == rhs.len() && self.iter().zip(rhs.iter()).all(|(a, b)| a == b)
	}
}

impl<'a, P: PartialEq> PartialEq<&'a [P]> for Path<P> {
	fn eq(&self, rhs: &&'a [P]) -> bool {
		// we can't just use slice's eq because self might be reversed
		self.len() == rhs.len() && self.iter().zip(rhs.iter()).all(|(a, b)| a == b)
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
