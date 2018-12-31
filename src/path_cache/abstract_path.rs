use crate::{generics::Path, neighbors::Neighborhood, NodeID, PathCache, Point};

pub struct AbstractPath<'a, N: Neighborhood> {
	src: &'a PathCache<N>,
	get_cost: Box<dyn Fn(Point) -> isize>,
	path: Path<NodeID>,
}

use std::fmt;
impl<N: Neighborhood> fmt::Debug for AbstractPath<'_, N> {
	fn fmt(&self, fmt: &mut fmt::Formatter) -> fmt::Result {
		fmt.debug_struct("AbstractPath")
			.field("path", &self.path)
			.finish()
	}
}
