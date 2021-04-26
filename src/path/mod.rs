mod abstract_path;
pub use abstract_path::AbstractPath;

mod generic_path;
pub use generic_path::*;

mod path_segment;
pub use path_segment::PathSegment;

pub type Cost = usize;
