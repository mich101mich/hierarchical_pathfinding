use crate::{generics::Path, NodeID, PathCache, Point};

pub struct AbstractPath<'a> {
	src: &'a PathCache,
	get_cost: Box<dyn Fn(Point) -> isize>,
	path: Path<NodeID>,
}

use std::fmt;
impl fmt::Debug for AbstractPath<'_> {
	fn fmt(&self, fmt: &mut fmt::Formatter) -> fmt::Result {
		fmt.debug_struct("AbstractPath")
			.field("path", &self.path)
			.finish()
	}
}
