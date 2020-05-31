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
}