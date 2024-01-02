use crate::{path::PathSegment, NodeIDMap, Point};

#[derive(Clone, Debug)]
pub(crate) struct Node {
    pub pos: Point,
    pub walk_cost: usize,
    pub edges: NodeIDMap<PathSegment>,
}

impl Node {
    pub fn new(pos: Point, walk_cost: usize) -> Node {
        Node {
            pos,
            walk_cost,
            edges: NodeIDMap::default(),
        }
    }
}
