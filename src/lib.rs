//! This is documentation for the `rusty_planner` crate.

#![doc = include_str!("../README.md")]
#![feature(binary_heap_retain)]

/// some helper routines (not public)
mod util;

/// Basic support for multi-agent distributed systems.
pub mod agent;
/// Module with some generic traits used by various algorithms.
pub mod planner;

/// Module implementing the Anytime Dynamic A* algorithm.
pub mod any_dyn_astar;
/// Module implementing the D* lite algorithm.
pub mod dstar_lite;
/// Module implementing an iterative repair algorithm.
pub mod iterative_repair;
/// Module implementing a UCT style Monte-Carlo Tree Search algorithm.
pub mod mcts;

/// Module implementing the Multi-Agent Distributed forward A* search algorithm.
pub mod mad_astar;
