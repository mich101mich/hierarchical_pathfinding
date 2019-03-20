#![allow(unused)]

pub trait IterExt<T>: Iterator<Item = T> {
	fn to_vec(self) -> Vec<T>;
}
impl<T, I: Iterator<Item = T>> IterExt<T> for I {
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
	pub fn all() -> impl Iterator<Item = Dir> {
		[UP, RIGHT, DOWN, LEFT].iter().cloned()
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
	($type:tt) => {
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
		impl Into<$type> for Dir {
			fn into(self) -> $type {
				self as $type
			}
		}
	};
}

impl_from_into!(u8);
impl_from_into!(u16);
impl_from_into!(u32);
impl_from_into!(u64);
impl_from_into!(usize);
impl_from_into!(i8);
impl_from_into!(i16);
impl_from_into!(i32);
impl_from_into!(i64);
impl_from_into!(isize);

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
	let diff = UNIT_CIRCLE[dir.num()];
	let diff = (diff.0 * dist as isize, diff.0 * dist as isize);
	let res = (pos.0 as isize + diff.0, pos.1 as isize + diff.1);
	if (res.0 >= base.0 as isize
		&& res.1 >= base.1 as isize
		&& res.0 < base.0 as isize + w as isize
		&& res.1 < base.1 as isize + h as isize)
	{
		None
	} else {
		Some((res.0 as usize, res.1 as usize))
	}
}
