use std::hash;

///
/// Public trait which - once implemented - describes the problem space to solve.
///
pub trait ProblemSpace {
    /// Defines the type of your state.
    type State: Copy + Eq + hash::Hash;
    /// Iterator for containing states and cost/utility to transition to it.
    type Iter: Iterator<Item=(Self::State, f64)>;  // action_id & cost.

    /// Heuristic function to calculate the "distance" between two states.
    fn heuristic(&self, _: &Self::State, _: &Self::State) -> f64;
    /// Given a state calculates the successor states.
    fn succ(&self, _: &Self::State) -> Self::Iter;
    /// Given a state calculates the predecessor states.
    fn pred(&self, _: &Self::State) -> Self::Iter;
    /// Called when obstacle was detected, or start state transitioned.
    fn update(&mut self, _: &Self::State) {}
    // TODO: check if we want to add callback here.
}

#[cfg(test)]
mod tests {
    use std::vec;

    use crate::planner;
    use crate::planner::ProblemSpace;

    struct Environment {}

    impl planner::ProblemSpace for Environment {
        type State = usize;
        type Iter = vec::IntoIter<(Self::State, f64)>;
        fn heuristic(&self, _: &Self::State, _: &Self::State) -> f64 { 0.0 }
        fn succ(&self, _: &Self::State) -> Self::Iter { vec![].into_iter() }
        fn pred(&self, s: &Self::State) -> Self::Iter { self.succ(s) }
    }

    // Test for success.

    #[test]
    fn test_update_for_success() {
        let mut env = Environment {};
        env.update(&1);
    }

    #[test]
    fn test_trait_for_success() {
        let mut env = Environment {};
        env.update(&1);
        env.heuristic(&0, &1);
        env.succ(&0);
        env.pred(&0);
    }
}
