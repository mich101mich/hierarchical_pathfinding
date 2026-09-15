use super::*;

/// A segment of a path. Might itself consist of more segments.
#[derive(Debug, Clone)]
pub struct PathSegment {
    /// The actual path, or a list of subsegments.
    pub(super) inner: InnerPath,
    /// The cost of traversing this path segment.
    pub(super) cost: usize,
    /// The length of this path segment.
    pub(super) len: usize,
}

#[derive(Debug, Clone)]
pub(super) enum InnerPath {
    /// A direct path through the grid.
    Raw(Vec<Point>),
    /// A path through a superchunk, consisting of multiple subsegments.
    Super(Vec<Arc<PathSegment>>),
}

#[derive(Debug)]
pub struct PathSegmentIter<'a> {
    inner: InnerPathSegmentIter<'a>,
    remaining: usize,
}
type InnerPathSegmentSuperIter<'a> = std::iter::FlatMap<
    std::slice::Iter<'a, Arc<PathSegment>>,
    PathSegmentIter<'a>,
    fn(&'a Arc<PathSegment>) -> PathSegmentIter<'a>,
>;

#[derive(Debug)]
enum InnerPathSegmentIter<'a> {
    Raw(std::slice::Iter<'a, Point>),
    Super(Box<InnerPathSegmentSuperIter<'a>>),
}

impl Iterator for PathSegmentIter<'_> {
    type Item = Point;

    fn next(&mut self) -> Option<Self::Item> {
        let ret = match &mut self.inner {
            InnerPathSegmentIter::Raw(iter) => iter.next().copied(),
            InnerPathSegmentIter::Super(iter) => iter.next(),
        };
        if ret.is_some() {
            self.remaining -= 1;
        }
        ret
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.remaining, Some(self.remaining))
    }
}

impl std::iter::FusedIterator for PathSegmentIter<'_> {}
impl ExactSizeIterator for PathSegmentIter<'_> {
    fn len(&self) -> usize {
        self.remaining
    }
}

impl PathSegment {
    #[must_use]
    pub fn cost(&self) -> usize {
        self.cost
    }

    #[must_use]
    pub fn length(&self) -> usize {
        self.len
    }

    #[must_use]
    pub fn iter(&self) -> PathSegmentIter<'_> {
        PathSegmentIter {
            inner: match &self.inner {
                InnerPath::Raw(points) => {
                    let mut iter = points.iter();
                    iter.next();
                    InnerPathSegmentIter::Raw(iter)
                }
                InnerPath::Super(segments) => {
                    InnerPathSegmentIter::Super(Box::new(segments.iter().flat_map(|s| s.iter())))
                }
            },
            remaining: self.len,
        }
    }
}

impl<'a> IntoIterator for &'a PathSegment {
    type Item = Point;
    type IntoIter = PathSegmentIter<'a>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}
