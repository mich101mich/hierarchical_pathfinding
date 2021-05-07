use super::Cost;

#[cfg(not(target_arch = "wasm32"))]
use std::sync::Arc;

#[cfg(target_arch = "wasm32")]
use std::rc::Rc as Arc; // TODO: What does Arc do on wasm?

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Path<P> {
    path: Arc<[P]>,
    cost: Cost,
    is_reversed: bool,
}

#[allow(dead_code)]
impl<P> Path<P> {
    pub fn new(path: Vec<P>, cost: Cost) -> Path<P> {
        Path {
            path: path.into(),
            cost,
            is_reversed: false,
        }
    }

    pub fn from_slice(path: &[P], cost: Cost) -> Path<P>
    where
        P: Clone,
    {
        Path {
            path: path.into(),
            cost,
            is_reversed: false,
        }
    }

    pub fn cost(&self) -> Cost {
        self.cost
    }

    pub fn len(&self) -> usize {
        self.path.len()
    }

    pub fn is_empty(&self) -> bool {
        self.path.is_empty()
    }

    pub fn reversed(&self, start_cost: Cost, end_cost: Cost) -> Path<P> {
        Path {
            path: self.path.clone(),
            cost: self.cost - start_cost + end_cost,
            is_reversed: !self.is_reversed,
        }
    }

    /// Returns an Iterator over the Path
    pub fn iter(&self) -> Iter<P> {
        Iter {
            iter: self.path.iter(),
            reversed: self.is_reversed,
        }
    }
}

use std::ops::Index;

impl<P> Index<usize> for Path<P> {
    type Output = P;
    fn index(&self, index: usize) -> &P {
        let index = if self.is_reversed {
            self.path.len() - index - 1
        } else {
            index
        };
        &self.path[index]
    }
}

#[derive(Debug)]
pub struct Iter<'a, P> {
    iter: std::slice::Iter<'a, P>,
    reversed: bool,
}

impl<'a, P> Iterator for Iter<'a, P> {
    type Item = &'a P;
    fn next(&mut self) -> Option<Self::Item> {
        if self.reversed {
            self.iter.next_back()
        } else {
            self.iter.next()
        }
    }
    fn size_hint(&self) -> (usize, Option<usize>) {
        self.iter.size_hint()
    }
}

impl<P> DoubleEndedIterator for Iter<'_, P> {
    fn next_back(&mut self) -> Option<Self::Item> {
        if self.reversed {
            self.iter.next()
        } else {
            self.iter.next_back()
        }
    }
}
impl<P> ExactSizeIterator for Iter<'_, P> {}
impl<P> std::iter::FusedIterator for Iter<'_, P> {}

impl<P: PartialEq> PartialEq<Vec<P>> for Path<P> {
    fn eq(&self, rhs: &Vec<P>) -> bool {
        // we can't just use slice's eq because self might be reversed
        self.len() == rhs.len() && self.iter().zip(rhs.iter()).all(|(a, b)| a == b)
    }
}

impl<'a, P: PartialEq> PartialEq<&'a [P]> for Path<P> {
    fn eq(&self, rhs: &&'a [P]) -> bool {
        // we can't just use slice's eq because self might be reversed
        self.len() == rhs.len() && self.iter().zip(rhs.iter()).all(|(a, b)| a == b)
    }
}

use std::cmp::Ordering;

impl<P: Eq> Ord for Path<P> {
    fn cmp(&self, other: &Path<P>) -> Ordering {
        self.cost.cmp(&other.cost)
    }
}

impl<P: PartialEq> PartialOrd for Path<P> {
    fn partial_cmp(&self, other: &Path<P>) -> Option<Ordering> {
        Some(self.cost.cmp(&other.cost))
    }
}

use std::fmt;
impl<P: fmt::Display> fmt::Display for Path<P> {
    fn fmt(&self, fmt: &mut fmt::Formatter) -> fmt::Result {
        write!(fmt, "Path[Cost = {}]: ", self.cost)?;
        if self.path.is_empty() {
            write!(fmt, "<empty>")
        } else {
            write!(fmt, "{}", self.path[0])?;
            for p in self.path.iter().skip(1) {
                write!(fmt, " -> {}", p)?;
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {

    use super::Path;
    #[test]
    fn index() {
        let path = Path::new(vec![4, 2, 0], 42);

        assert_eq!(path[0], 4);
        assert_eq!(path[1], 2);
        assert_eq!(path[2], 0);
    }

    #[test]
    fn display() {
        let path = Path::new(vec![4, 2, 0], 42);

        assert_eq!(&format!("{}", path), "Path[Cost = 42]: 4 -> 2 -> 0");
    }

    #[test]
    fn display_empty() {
        let path = Path::new(Vec::<i32>::new(), 0);

        assert_eq!(&format!("{}", path), "Path[Cost = 0]: <empty>");
    }
}
