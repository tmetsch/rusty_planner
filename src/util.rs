use std::cmp;

/// rhs & g data for A* related searches...
pub struct StateData {
    pub rhs: f64,
    pub g: f64,
}

/// Entry in the Min-heap.
pub struct HeapEntry<S> {
    pub state: S,
    pub keys: (f64, f64),
}

impl<S> HeapEntry<S> {
    pub fn new_entry(state: S, keys: (f64, f64)) -> HeapEntry<S> {
        HeapEntry {
            state,
            keys,
        }
    }
}

impl<S> PartialEq for HeapEntry<S> {
    fn eq(&self, other: &HeapEntry<S>) -> bool {
        let s_k0 = (self.keys.0 * 1024.0 * 1024.0).round() as i64;
        let s_k1 = (self.keys.1 * 1024.0 * 1024.0).round() as i64;
        let o_k0 = (other.keys.0 * 1024.0 * 1024.0).round() as i64;
        let o_k1 = (other.keys.1 * 1024.0 * 1024.0).round() as i64;
        (s_k0, s_k1) == (o_k0, o_k1)
    }
}

impl<S> Eq for HeapEntry<S> {}

impl<S> PartialOrd for HeapEntry<S> {
    fn partial_cmp(&self, other: &HeapEntry<S>) -> Option<cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<S> Ord for HeapEntry<S> {
    fn cmp(&self, other: &HeapEntry<S>) -> cmp::Ordering {
        // Lexigraphic order - first k_0, otherwise k_1
        let s_k0 = (self.keys.0 * 1024.0 * 1024.0).round() as i64;
        let s_k1 = (self.keys.1 * 1024.0 * 1024.0).round() as i64;
        let o_k0 = (other.keys.0 * 1024.0 * 1024.0).round() as i64;
        let o_k1 = (other.keys.1 * 1024.0 * 1024.0).round() as i64;
        o_k0.cmp(&s_k0).then_with(|| o_k1.cmp(&s_k1))
    }
}

#[cfg(test)]
mod tests {
    use std::collections;

    use crate::util;

    #[test]
    fn test_queueentry_for_sanity() {
        let mut open: collections::BinaryHeap<util::HeapEntry<String>> =
            collections::BinaryHeap::new();
        let a = String::from("A");
        let b = String::from("B");
        let c = String::from("C");
        let d = String::from("D");
        let s_0 = util::HeapEntry::new_entry(a, (1.0, 1.0));
        let s_1 = util::HeapEntry::new_entry(b, (1.0, 0.0));
        let s_2 = util::HeapEntry::new_entry(c, (2.0, 1.0));
        let s_3 = util::HeapEntry::new_entry(d, (1.0, 2.0));
        open.push(s_0);
        open.push(s_1);
        open.push(s_2);
        open.push(s_3);
        // make sure lexicographic order works for priorities...
        assert_eq!(open.peek().unwrap().state, "B");
        assert_eq!(open.pop().unwrap().keys, (1.0, 0.0));
        assert_eq!(open.pop().unwrap().keys, (1.0, 1.0));
        assert_eq!(open.pop().unwrap().keys, (1.0, 2.0));
        assert_eq!(open.pop().unwrap().keys, (2.0, 1.0));
    }
}
