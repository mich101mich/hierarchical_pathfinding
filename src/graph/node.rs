use crate::{Point, path::PathSegment};
use super::{NodeID, NodeIDMap};

#[derive(Clone, Debug)]
pub struct Node {
    pub id: NodeID,
    pub pos: Point,
    pub walk_cost: usize,
    pub edges: NodeIDMap<PathSegment>,
}

impl Node {
    pub fn new(id: NodeID, pos: Point, walk_cost: usize) -> Node {
        Node {
            id,
            pos,
            walk_cost,
            edges: NodeIDMap::default(),
        }
    }
}
