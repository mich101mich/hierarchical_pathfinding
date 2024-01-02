use super::{Node, NodeID, NodeIDMap, NodeIDSet};
use crate::{path::PathSegment, Point, PointMap};

#[derive(Clone, Debug)]
pub(crate) struct NodeList {
    nodes: slab::Slab<Node>,
    pos_map: PointMap<NodeID>,
}

impl NodeList {
    pub fn new() -> Self {
        Self {
            nodes: slab::Slab::default(),
            pos_map: PointMap::default(),
        }
    }

    pub fn len(&self) -> usize {
        self.pos_map.len()
    }

    pub fn add_node(&mut self, pos: Point, walk_cost: usize) -> NodeID {
        let id = self.nodes.insert(Node::new(pos, walk_cost));
        self.pos_map.insert(pos, id);
        id
    }

    pub fn add_edge(&mut self, src: NodeID, target: NodeID, path: PathSegment) {
        let src_node = &self[src];
        if let Some(existing) = src_node.edges.get(&target) {
            if existing.cost() == path.cost() {
                return;
            }
        }
        let src_cost = src_node.walk_cost;

        let target_node = &mut self[target];

        let target_cost = target_node.walk_cost;

        let other_path = path.reversed(src_cost, target_cost);
        target_node.edges.insert(src, other_path);

        let src_node = &mut self[src];
        src_node.edges.insert(target, path);
    }

    #[track_caller]
    pub fn remove_node(&mut self, id: NodeID) {
        let node = self.nodes.remove(id);
        for (other_id, _) in node.edges {
            self[other_id].edges.remove(&id);
        }
        self.pos_map.remove(&node.pos);
    }

    pub fn iter(&self) -> slab::Iter<Node> {
        self.nodes.iter()
    }

    pub fn id_at(&self, pos: Point) -> Option<NodeID> {
        self.pos_map.get(&pos).copied()
    }

    pub fn absorb(&mut self, other: NodeList) -> NodeIDSet {
        let mut ret = NodeIDSet::default();
        let mut map = NodeIDMap::default();

        for (old, node) in other.nodes.iter() {
            let new = self.add_node(node.pos, node.walk_cost);
            map.insert(old, new);
            ret.insert(new);
        }

        for (id, old_node) in other.nodes {
            let new_node = &mut self[map[&id]];
            new_node.edges = old_node
                .edges
                .into_iter()
                .map(|(other_id, path)| (map[&other_id], path))
                .collect();
        }

        ret
    }
}

use std::ops::{Index, IndexMut};
impl Index<NodeID> for NodeList {
    type Output = Node;
    #[track_caller]
    fn index(&self, index: NodeID) -> &Node {
        &self.nodes[index]
    }
}
impl IndexMut<NodeID> for NodeList {
    #[track_caller]
    fn index_mut(&mut self, index: NodeID) -> &mut Node {
        &mut self.nodes[index]
    }
}

#[test]
fn absorb() {
    let mut nodes = NodeList::new();
    let zero_id = nodes.add_node((0, 0), 0);
    let one_id = nodes.add_node((1, 1), 1);
    let two_id = nodes.add_node((2, 2), 2);
    nodes.add_edge(
        zero_id,
        one_id,
        PathSegment::new(super::Path::from_slice(&[], 0), true),
    );
    nodes.add_edge(
        two_id,
        zero_id,
        PathSegment::new(super::Path::from_slice(&[], 2), true),
    );

    let mut new_nodes = NodeList::new();
    let ten_id = new_nodes.add_node((10, 10), 10);
    let eleven_id = new_nodes.add_node((11, 11), 11);
    new_nodes.add_edge(
        ten_id,
        eleven_id,
        PathSegment::new(super::Path::from_slice(&[], 10), true),
    );

    nodes.absorb(new_nodes);

    let new_ten_id = nodes.id_at((10, 10)).unwrap();
    let new_eleven_id = nodes.id_at((11, 11)).unwrap();

    assert_eq!(nodes.nodes.len(), 5);
    assert_eq!(nodes.nodes[new_ten_id].pos, (10, 10));
    assert_eq!(nodes.nodes[new_eleven_id].pos, (11, 11));
    assert_eq!(nodes.nodes[new_ten_id].edges[&new_eleven_id].cost(), 10);
}
