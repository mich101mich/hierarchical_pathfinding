use super::*;

impl<T: Send + Sync + 'static> DensePathCache<T> {
    pub(super) fn find_path_impl(
        &self,
        start: (usize, usize),
        end: (usize, usize),
    ) -> Option<PathSegment> {
        todo!();
    }

    pub(super) fn find_all_paths_impl(
        &self,
        start: (usize, usize),
        ends: &[(usize, usize)],
    ) -> Vec<Option<PathSegment>> {
        todo!();
    }

    pub(super) fn find_any_path_impl(
        &self,
        start: (usize, usize),
        ends: &[(usize, usize)],
    ) -> Option<PathSegment> {
        todo!();
    }
}
