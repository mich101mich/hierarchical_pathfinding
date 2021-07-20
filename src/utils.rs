#![allow(unused)]

#[allow(clippy::wrong_self_convention)]
pub trait IterExt<T>: Iterator<Item = T> {
    fn to_vec(self) -> Vec<T>;
}
impl<T, I: Iterator<Item = T>> IterExt<T> for I {
    #[allow(clippy::wrong_self_convention)]
    fn to_vec(self) -> Vec<T> {
        self.collect()
    }
}

use crate::Point;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dir {
    UP = 0,
    RIGHT = 1,
    DOWN = 2,
    LEFT = 3,
}
pub use self::Dir::*;

impl Dir {
    pub fn all() -> std::iter::Copied<std::slice::Iter<'static, Dir>> {
        [UP, RIGHT, DOWN, LEFT].iter().copied()
    }
    pub fn opposite(self) -> Dir {
        ((self.num() + 2) % 4).into()
    }
    pub fn num(self) -> usize {
        self as usize
    }
    pub fn is_vertical(self) -> bool {
        self == UP || self == DOWN
    }
}

macro_rules! impl_from_into {
    ($($type:ty),+) => {$(
        impl From<$type> for Dir {
            fn from(val: $type) -> Dir {
                match val {
                    0 => UP,
                    1 => RIGHT,
                    2 => DOWN,
                    3 => LEFT,
                    _ => panic!("invalid Dir: {}", val),
                }
            }
        }
        impl From<Dir> for $type {
            fn from(dir: Dir) -> $type {
                dir as $type
            }
        }
    )+}
}

impl_from_into!(u8, u16, u32, u64, usize, i8, i16, i32, i64, isize);

const UNIT_CIRCLE: [(isize, isize); 4] = [(0, -1), (1, 0), (0, 1), (-1, 0)];

pub fn get_in_dir(pos: Point, dir: Dir, base: Point, (w, h): (usize, usize)) -> Option<Point> {
    let diff = UNIT_CIRCLE[dir.num()];
    if (pos.0 == base.0 && diff.0 < 0)
        || (pos.1 == base.1 && diff.1 < 0)
        || (pos.0 == base.0 + w - 1 && diff.0 > 0)
        || (pos.1 == base.1 + h - 1 && diff.1 > 0)
    {
        None
    } else {
        Some((
            (pos.0 as isize + diff.0) as usize,
            (pos.1 as isize + diff.1) as usize,
        ))
    }
}

pub fn jump_in_dir(
    pos: Point,
    dir: Dir,
    dist: usize,
    base: Point,
    (w, h): (usize, usize),
) -> Option<Point> {
    let dist = dist as isize;
    let left = base.0 as isize;
    let right = (base.0 + w) as isize;
    let top = base.1 as isize;
    let bottom = (base.1 + h) as isize;

    let diff = UNIT_CIRCLE[dir.num()];
    let res = (
        pos.0 as isize + diff.0 * dist,
        pos.1 as isize + diff.1 * dist,
    );
    if res.0 >= left && res.0 < right && res.1 >= top && res.1 < bottom {
        Some((res.0 as usize, res.1 as usize))
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn jump_test() {
        let pos = (1, 3);
        assert_eq!(jump_in_dir(pos, UP, 2, (0, 0), (5, 5)), Some((1, 1)));
        assert_eq!(jump_in_dir(pos, RIGHT, 2, (0, 0), (5, 5)), Some((3, 3)));
        assert_eq!(jump_in_dir(pos, DOWN, 2, (0, 0), (5, 5)), None);
        assert_eq!(jump_in_dir(pos, LEFT, 2, (0, 0), (5, 5)), None);
    }
}
