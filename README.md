
# AI Planning & Reasoning library for Rust

This is a library written in [Rust](https://www.rust-lang.org/) containing Artificial Intelligence algorithms - in 
particular those from the field of planning & reasoning. The goal is to support the following algorithms in the long 
run:

  * Generic Planning Algorithms
     * M. Likhachev, D. Ferguson, G. Gordon, A. Stentz, and S. Thrun, **Anytime dynamic A\*: an anytime, replanning 
       algorithm**. Fifteenth International Conference on International Conference on Automated Planning and Scheduling
       (ICAPS’05), 2005, https://dl.acm.org/doi/10.5555/3037062.3037096
     * S. Koenig and M. Likhachev, **D\*lite**. Eighteenth national conference on Artificial intelligence, 2002,
       https://dl.acm.org/doi/10.5555/777092.777167
     * M. Zweben, E. Davis, B. Daun and M. J. Deale, **Scheduling and rescheduling with iterative repair**. IEEE 
       Transactions on Systems, Man, and Cybernetics, 1993, https://ieeexplore.ieee.org/document/257756
  * Multi-Agent Planning - Coordination, Negotiation/Bidding, Coalition Formation:
     * T. Sandholm, K. Larson, M. Andersson, O. Shehory, and F. Tohmé, **Coalition structure generation with worst case 
       guarantees**. Artif. Intell. 111, 1999, https://doi.org/10.1016/S0004-3702(99)00036-3
     * David Silver,  **Cooperative pathfinding**. First AAAI Conference on Artificial Intelligence and Interactive
       Digital Entertainment (AIIDE’05), 2005, https://dl.acm.org/doi/10.5555/3022473.3022494. 
     * R. Nissim and R. Brafman, **Distributed Heuristic Forward Search for Multi-Agent Systems**. arXiv, 2013. 
       https://arxiv.org/abs/1306.5858
     * S. Gelly, Y. Wang, R. Munos, and O. Teytaud **Modification of UCT with Patterns in Monte-Carlo Go**. Technical report, INRIA, 2006, https://doi.org/10.1007%2F11871842_29.
     * O. Shehory and S. Kraus, **Task allocation via coalition formation among autonomous agents**. 14th international
       joint conference on Artificial intelligence - Volume 1 (IJCAI’95), 1995, 
       https://dl.acm.org/doi/10.5555/1625855.1625941.

**_Note_**: This library is still work in progress - and mostly for me to get familiar with Rust. Use at 
your own risk; refactorings & bigger changes might still happen.  

## Motivation

While Deep Learning algorithms are currently a hot topic, it is also necessary to have planning & reasoning capabilities
to build truly autonomous systems. Be it for robots moving around, ways to automatically plan how ships should berth, 
in which order airplanes should land on airports, etc.. There are countless examples of how these algorithms can help 
solve real world problems & deliver efficient solutions. Recommended reads include 
[Artificial Intelligence: A Modern Approach](http://aima.cs.berkeley.edu/) by Stuart Russell & Peter Norvig; Furthermore 
NASA's JPL [Artificial Intelligence group](https://ai.jpl.nasa.gov/) has done tons of research in this area - especially 
their methods of controlling autonomous space probes using planners & execution components are interesting; Imagine how 
cool it would be if those concept would be embedded in future control planes...
