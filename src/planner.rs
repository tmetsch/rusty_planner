use std::hash;

// TODO: check usage of &str + lifetime vs String.
// TODO: check if we can have a generic PDDL parser --> ProblemSpace.

///
/// Public trait which - once implemented - describes the problem space to solve.
///
pub trait ProblemSpace {
    /// Defines the type of your state.
    type State: Copy + Eq + hash::Hash;
    /// Iterator for containing states and cost/utility to transition to it.
    type Iter: Iterator<Item = (Self::State, f64)>; // action_id & cost.

    /// Heuristic function to calculate the "distance" between two states.
    fn heuristic(&self, _: &Self::State, _: &Self::State) -> f64;
    /// Given a state calculates the successor states.
    fn succ(&self, _: &Self::State) -> Self::Iter;
    /// Given a state calculates the predecessor states.
    fn pred(&self, _: &Self::State) -> Self::Iter;
}

///
/// Trait for lifelong planning algorithms.
///
pub trait Lifelong: ProblemSpace {
    /// Called when obstacle was detected, or start state transitioned.
    fn update(&mut self, _: &Self::State) {}
}

///
/// Trait to enable anytime algorithms to signal back results.
///
pub trait Anytime: ProblemSpace {
    /// Callback to signal - for example - a partial result.
    fn callback(&mut self, _: &Self::State);
}

///
/// Trait for multi-agent/system planning algorithms.
///
pub trait SharedStates: ProblemSpace {
    /// Determine if a given state is a public state.
    fn is_public(&self, _: &Self::State) -> bool;
    /// Serialize a state into a string.
    /// * `msg_type` - Type of the message.
    /// * `state` - The state to serialize.
    /// * `para` - Vector for parameters such as g, rhs and h values.
    fn serialize(&self, msg_type: u8, _: &Self::State, _: Vec<f64>) -> String;
    /// Serialize a string into a state.
    fn deserialize(&self, _: String) -> (u8, Self::State, Vec<f64>);
}

#[cfg(test)]
mod tests {
    use std::vec;

    use crate::planner::Lifelong;
    use crate::planner::ProblemSpace;
    use crate::planner::SharedStates;

    struct Environment {}

    impl ProblemSpace for Environment {
        type State = usize;
        type Iter = vec::IntoIter<(Self::State, f64)>;
        fn heuristic(&self, _: &Self::State, _: &Self::State) -> f64 {
            0.0
        }
        fn succ(&self, _: &Self::State) -> Self::Iter {
            vec![].into_iter()
        }
        fn pred(&self, s: &Self::State) -> Self::Iter {
            self.succ(s)
        }
    }

    impl Lifelong for Environment {
        // use default...
    }

    impl SharedStates for Environment {
        fn is_public(&self, _: &Self::State) -> bool {
            false
        }
        fn serialize(&self, _: u8, _: &Self::State, _: Vec<f64>) -> String {
            String::from("0")
        }
        fn deserialize(&self, _: String) -> (u8, Self::State, Vec<f64>) {
            (0, 1, vec![])
        }
    }

    // Test for success.

    #[test]
    fn test_update_for_success() {
        let mut env = Environment {};
        env.update(&1);
    }

    #[test]
    fn test_traits_for_success() {
        let mut env = Environment {};
        env.update(&1);
        env.heuristic(&0, &1);
        env.succ(&0);
        env.pred(&0);
        env.is_public(&0);
        env.serialize(0, &0, vec![]);
        env.deserialize(String::from("0"));
    }
}
