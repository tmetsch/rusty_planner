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
        HeapEntry { state, keys }
    }
}

const KEY_SCALE: f64 = 1024.0 * 1024.0;

impl<S> PartialEq for HeapEntry<S> {
    fn eq(&self, other: &HeapEntry<S>) -> bool {
        let s_k0 = (self.keys.0 * KEY_SCALE).round();
        let s_k1 = (self.keys.1 * KEY_SCALE).round();
        let o_k0 = (other.keys.0 * KEY_SCALE).round();
        let o_k1 = (other.keys.1 * KEY_SCALE).round();

        // Use f64 comparison to avoid integer overflow
        (s_k0 - o_k0).abs() < f64::EPSILON && (s_k1 - o_k1).abs() < f64::EPSILON
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
        // Lexicographic order - first k_0, otherwise k_1
        // Scale and compare as f64 to maintain precision
        let s_k0 = self.keys.0 * KEY_SCALE;
        let s_k1 = self.keys.1 * KEY_SCALE;
        let o_k0 = other.keys.0 * KEY_SCALE;
        let o_k1 = other.keys.1 * KEY_SCALE;

        // Compare with epsilon tolerance
        if (s_k0 - o_k0).abs() > f64::EPSILON {
            o_k0.partial_cmp(&s_k0).unwrap()
        } else if (s_k1 - o_k1).abs() > f64::EPSILON {
            o_k1.partial_cmp(&s_k1).unwrap()
        } else {
            cmp::Ordering::Equal
        }
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
