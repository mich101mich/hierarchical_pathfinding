use super::{path_segment::PathSegment, Node};
use crate::{NodeID, Point};
use std::collections::HashMap;

macro_rules! invalid_id {
	() => {
		|| panic!("Invalid NodeID in {}:{}", file!(), line!())
	};
}

#[derive(Clone, Debug)]
pub struct NodeMap {
	nodes: HashMap<NodeID, Node>,
	next_id: NodeID,
}

impl NodeMap {
	pub fn new() -> NodeMap {
		NodeMap {
			nodes: HashMap::new(),
			next_id: 0,
		}
	}

	pub fn add_node(&mut self, pos: Point, walk_cost: isize) -> NodeID {
		let id = self.next_id;
		self.next_id += 1;

		let node = Node::new(id, pos, walk_cost);
		self.nodes.insert(id, node);
		id
	}

	pub fn add_edge(&mut self, src: NodeID, target: NodeID, path: PathSegment) {
		let (src_pos, src_cost) = {
			let node = &self[&src];
			(node.pos, node.walk_cost)
		};
		assert!(src_cost >= 0, "Cannot add Path from solid Node");

		let target_node = self.get_mut(&target).unwrap_or_else(invalid_id!());

		let target_pos = target_node.pos;
		let target_cost = target_node.walk_cost;

		if target_cost >= 0 {
			let other_path = path.reversed(src_cost as usize, target_cost as usize);
			target_node.edges.insert(src, other_path);
		} else {
			target_node.edges.insert(
				src,
				PathSegment::Unknown {
					start: src_pos,
					end: target_pos,
					cost: std::usize::MAX,
					len: std::usize::MAX,
				},
			);
		}

		let src_node = self.get_mut(&src).unwrap_or_else(invalid_id!());
		src_node.edges.insert(target, path);
	}

	pub fn remove_node(&mut self, id: NodeID) {
		let node = self.remove(&id).unwrap_or_else(invalid_id!());
		for (other_id, _) in node.edges {
			self.get_mut(&other_id)
				.unwrap_or_else(invalid_id!())
				.edges
				.remove(&id);
		}
	}
}

use std::ops::{Deref, DerefMut};
impl Deref for NodeMap {
	type Target = HashMap<NodeID, Node>;
	fn deref(&self) -> &HashMap<NodeID, Node> {
		&self.nodes
	}
}
impl DerefMut for NodeMap {
	fn deref_mut(&mut self) -> &mut HashMap<NodeID, Node> {
		&mut self.nodes
	}
}
