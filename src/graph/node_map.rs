use super::{Node, NodeID};
use crate::{path::PathSegment, Point};

macro_rules! invalid_id {
	() => {
		|| panic!("Invalid NodeID in {}:{}", file!(), line!())
	};
}

#[derive(Clone, Debug)]
pub struct NodeMap {
	nodes: Vec<Option<Node>>,
	next_id: usize,
}

impl NodeMap {
	pub fn new() -> NodeMap {
		NodeMap {
			nodes: Vec::new(),
			next_id: 0,
		}
	}

	pub fn add_node(&mut self, pos: Point, walk_cost: isize) -> NodeID {
		while self.next_id < self.nodes.len() && self.nodes[self.next_id].is_some() {
			self.next_id += 1;
		}
		let id = self.next_id;
		self.next_id += 1;

		let node = Node::new(id as NodeID, pos, walk_cost);
		if id >= self.nodes.len() {
			self.nodes.push(Some(node));
		} else {
			self.nodes[id] = Some(node);
		}
		id as NodeID
	}

	pub fn add_edge(&mut self, src: NodeID, target: NodeID, path: PathSegment) {
		let (src_pos, src_cost) = {
			let node = &self[src];
			(node.pos, node.walk_cost)
		};
		assert!(src_cost >= 0, "Cannot add Path from solid Node");

		let target_node = &mut self[target];

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

		let src_node = &mut self[src];
		src_node.edges.insert(target, path);
	}

	pub fn remove_node(&mut self, id: NodeID) {
		let node = self.nodes[id as usize].take().unwrap_or_else(invalid_id!());
		for (other_id, _) in node.edges {
			self[other_id].edges.remove(&id);
		}
	}

	pub fn iter(&self) -> impl Iterator<Item = (NodeID, &Node)> + '_ {
		self.nodes
			.iter()
			.enumerate()
			.filter_map(|(id, opt)| opt.as_ref().map(|node| (id as NodeID, node)))
	}
	pub fn keys(&self) -> impl Iterator<Item = NodeID> + '_ {
		self.nodes
			.iter()
			.enumerate()
			.filter(|(_, opt)| opt.is_some())
			.map(|(id, _)| id as NodeID)
	}
	pub fn values(&self) -> impl Iterator<Item = &Node> + '_ {
		self.nodes.iter().filter_map(|opt| opt.as_ref())
	}

	pub fn find_id<F: FnMut(&Node) -> bool>(&self, mut f: F) -> Option<NodeID> {
		self.iter().find(|(_, node)| f(node)).map(|(id, _)| id)
	}
	pub fn id_at(&self, pos: Point) -> Option<NodeID> {
		self.find_id(|n| n.pos == pos)
	}
}

use std::ops::{Index, IndexMut};
impl Index<NodeID> for NodeMap {
	type Output = Node;
	#[track_caller]
	fn index(&self, index: NodeID) -> &Node {
		self.nodes[index as usize].as_ref().unwrap()
	}
}
impl IndexMut<NodeID> for NodeMap {
	#[track_caller]
	fn index_mut(&mut self, index: NodeID) -> &mut Node {
		self.nodes[index as usize].as_mut().unwrap()
	}
}
