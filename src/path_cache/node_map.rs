use super::{path_segment::PathSegment, Node};
use crate::{NodeID, Point};
use std::collections::HashMap;

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
		let src_cost = self[&src].walk_cost;
		let target_cost = self[&target].walk_cost;

		assert!(src_cost >= 0, "Cannot add Path from solid Node");

		if target_cost >= 0 {
			let other_path = path.reversed(src_cost as usize, target_cost as usize);
			let node = self.get_mut(&target).unwrap();
			node.edges.insert(src, other_path);
		}

		let node = self.get_mut(&src).unwrap();
		node.edges.insert(target, path);
	}

	pub fn remove_node(&mut self, id: NodeID) {
		let node = self.remove(&id).unwrap();
		for (other_id, _) in node.edges {
			self.get_mut(&other_id).unwrap().edges.remove(&id);
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
