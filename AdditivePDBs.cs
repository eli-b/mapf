using System.Collections.Generic;
using System.Diagnostics;

namespace CPF_experiment
{
    public class AdditivePDBs : HeuristicCalculator
    {
        List<PDB> m_vPDBs; 

        /**
         * The set of agents that are not covered by this set of pattern
         * databases. This information is not used by this class or any of its
         * related classes, but is simply precomputed as useful information for
         * the caller (for example, if the caller wanted to add to our
         * heuristic estimate).
         */

        public SortedSet<uint> m_vExcludedAgents;

        /// <summary>
        /// Determines how many additive pattern databases to build and divides
        /// the agents among them, possibly leaving some agents out.
        /// </summary>
        /// <param name="s">The root of the search tree. This is also expected
        /// to have context parameters such as agent goal states.</param>

        public void build(ProblemInstance pi, WorldState s)
        {
            Debug.Write("Building database...");

            /**
             * As a simple rule, we'll simply take pairs of agents starting
             * with the first two, then the second two, etc.
             */

            m_vPDBs = new List<PDB>();
            if (s.allAgentsState.Length > 1)
            {
                for (uint i = 0; i < s.allAgentsState.Length - 1; i += 2)
                {

                    /**
                     * Make a list of agents we want to include together in the
                     * next additive pattern database. We specify agents by
                     * their index into the Travor_WorldState.allAgentsState
                     * array.
                     */

                    List<uint> vAgents = new List<uint>();
                    vAgents.Add(i);
                    vAgents.Add(i + 1);

                    /**
                     * Create a new root search node where the state only
                     * includes a subset of the agents of the original search
                     * node. This is done by passing into the state copy
                     * constructor our list of important agents.
                     */

                    WorldState tws = new WorldState(s.allAgentsState, vAgents);

                    /**
                     * Initialize, build, and save the new pattern database.
                     */

                    EnumeratedPDB pdb = new EnumeratedPDB();
                    pdb.init(pi, vAgents);
                    pdb.build();
                    Debug.Write(".");
                    m_vPDBs.Add(pdb);
                }
            }

            /**
             * Create single shortest path pattern database heuristics for the
             * remaining agents if we have any left over.
             */

            if (s.allAgentsState.Length % 2 == 1)
            {
                SingleShortestPath pdb = new SingleShortestPath();
                List<uint> vAgents = new List<uint>(1);
                vAgents.Add((uint) s.allAgentsState.Length - 1);
                pdb.init(pi, vAgents);
                pdb.build();
                m_vPDBs.Add(pdb);
            }

            /**
             * For informational purposes, we will set the number of agents
             * that aren't included in this set of pattern databases.
             */

            m_vExcludedAgents = new SortedSet<uint>();

            Debug.WriteLine("done.");
        }

        /// <summary>
        /// Initializes the pattern database by storing references to the
        /// problem instance and also the subset of agents that the pattern
        /// database pertains to.
        /// </summary>
        /// <param name="pi">The problem instance.</param>
        /// <param name="vAgents">The agents that the pattern database should keep track of.</param>

        public virtual void init(ProblemInstance pi, List<uint> vAgents)
        {}

        /// <summary>
        /// Simply returns the sum of each of the additive pattern database 
        /// heuristic estimates on the given state.
        /// </summary>
        /// <param name="s">The state.</param>
        /// <returns>The admissible heuristic value for the additive pattern
        /// databases.</returns>

        public uint h(WorldState s)
        {
            uint nHeuristicValue = 0;
            foreach (PDB p in m_vPDBs)
            {
                nHeuristicValue += p.h(s);
            }
            return (nHeuristicValue);
        }

        public bool empty()
        {
            return (m_vPDBs.Count == 0);
        }
    }
}
