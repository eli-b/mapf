using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CPF_experement
{
    public interface HeuristicCalculator
    {
        uint h(WorldState s);


        /// <summary>
        /// Initializes the pattern database by storing references to the
        /// problem instance and also the subset of agents that the pattern
        /// database pertains to.
        /// </summary>
        /// <param name="pi">The problem instance.</param>
        /// <param name="vAgents">The agents that the pattern database should keep track of.</param>
        void init(ProblemInstance pi, List<uint> vAgents);
    }
}
