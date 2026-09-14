use super::*;

impl Dir {
    pub(super) fn step(self, (x, y): Point, bounds: (usize, usize)) -> Option<Point> {
        let (w, h) = bounds;
        match self {
            Dir::Up => Some((x, y.checked_sub(1)?)),
            Dir::Right => (x + 1 < w).then_some((x + 1, y)),
            Dir::Down => (y + 1 < h).then_some((x, y + 1)),
            Dir::Left => Some((x.checked_sub(1)?, y)),
        }
    }
}

pub(super) const fn to_chunk_pos((x, y): (usize, usize)) -> (usize, usize) {
    (x / CHUNK_SIZE, y / CHUNK_SIZE)
}

impl Chunk {
    #[track_caller]
    pub(super) fn exit_at(&self, (x, y): Point) -> &Option<Exit> {
        if y == 0 {
            self.top_exits.get(x)
        } else if y == CHUNK_SIZE - 1 {
            self.bottom_exits.get(x)
        } else if x == 0 {
            self.left_exits.get(y - 1)
        } else if x == CHUNK_SIZE - 1 {
            self.right_exits.get(y - 1)
        } else {
            panic!("Invalid exit position");
        }
        .unwrap()
    }

    #[track_caller]
    pub(super) fn exit_at_mut(&mut self, (x, y): Point) -> &mut Option<Exit> {
        if y == 0 {
            self.top_exits.get_mut(x)
        } else if y == CHUNK_SIZE - 1 {
            self.bottom_exits.get_mut(x)
        } else if x == 0 {
            self.left_exits.get_mut(y - 1)
        } else if x == CHUNK_SIZE - 1 {
            self.right_exits.get_mut(y - 1)
        } else {
            panic!("Invalid exit position");
        }
        .unwrap()
    }

    pub(super) fn border_iterator(&self) -> impl Iterator<Item = (Point, &Option<Exit>)> {
        let right = CHUNK_SIZE - 1;
        let bottom = CHUNK_SIZE - 1;

        let top_row = (0..=right).map(move |x| ((x, 0), &self.top_exits[x]));
        let bottom_row = (0..=right).map(move |x| ((x, bottom), &self.bottom_exits[x]));
        let right_col = (1..bottom).map(move |y| ((right, y), &self.right_exits[y - 1]));
        let left_col = (1..bottom).map(move |y| ((0, y), &self.left_exits[y - 1]));

        top_row.chain(bottom_row).chain(left_col).chain(right_col)
    }

    pub(super) fn border_iterator_mut(
        &mut self,
    ) -> impl Iterator<Item = (Point, &mut Option<Exit>)> {
        let right = CHUNK_SIZE - 1;
        let bottom = CHUNK_SIZE - 1;

        let top_row = self
            .top_exits
            .iter_mut()
            .enumerate()
            .map(move |(x, exit)| ((x, 0), exit));
        let bottom_row = self
            .bottom_exits
            .iter_mut()
            .enumerate()
            .map(move |(x, exit)| ((x, bottom), exit));
        let right_col = self
            .right_exits
            .iter_mut()
            .enumerate()
            .map(move |(y, exit)| ((right, y + 1), exit));
        let left_col = self
            .left_exits
            .iter_mut()
            .enumerate()
            .map(move |(y, exit)| ((0, y + 1), exit));

        top_row.chain(bottom_row).chain(left_col).chain(right_col)
    }
}
