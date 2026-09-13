use super::*;

impl<T: Send + Sync + 'static> DensePathCache<T> {
    pub(super) fn update_cache_impl(&mut self) {
        todo!();
        self.dirty_chunks.clear();
    }
}
