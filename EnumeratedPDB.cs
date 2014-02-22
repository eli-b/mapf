using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
using System.Diagnostics;

namespace CPF_experiment
{

    /// <summary>
    /// Represents a pattern database that enumerates all possible
    /// configurations of the state space by mapping each state to a unique
    /// 32-bit integer. A lookup table in memory maps each 32-bit integer
    /// representation of the state to a heuristic estimate. In order to reduce
    /// memory consumption even further, the heuristic estimates are internally
    /// stored as the difference on top of the single shortest path heuristic.
    /// The hope is that the reduced magnitude of heuristic values will now
    /// always fit within the range of a single unsigned byte. This assumption,
    /// of course, will break on instances that are large enough to cause an
    /// overflow.
    /// </summary>

    [Serializable]
    class EnumeratedPDB : PDB
    {

        /// <summary>
        /// The index is the hash of the state, and the resulting value should
        /// be added to that reported by the true path heuristic in order to
        /// get the correct admissible heuristic value. We reserve Byte.MaxValue
        /// to represent an uninitialized table entry.
        /// </summary>
        Byte[] m_vTable;

        /// <summary>
        /// m_vPermutations[i] represents the number of permutations of the
        /// remaining agents after placing the ith agent. For example, if we
        /// had 10 agents, and we've already placed 3 of them, then
        /// m_vPermutations[2] represents the number of permutations for the
        /// remaining 7 agents. This precomputed table depends on the number
        /// of free locations in the board and is used as a perfect hash 
        /// function that maps a state in the search space to an integer.
        /// </summary>
        UInt64[] m_vPermutations;

        /// <summary>
        /// Determines whether or not we will internally represent heuristic
        /// values as offsets from the single shortest path heuristic.
        /// </summary>
        bool m_bOffsetFromSingleShortestPath = true;

        /// <summary>
        /// Encapsulates the two file streams we will be using as we are
        /// building the pattern database. These two file streams represent the
        /// nodes we will be reading for input, and the resulting generated
        /// children nodes which will be outputted to the second file. We also
        /// include certain measures such as the number of nodes that have been
        /// written to a file, and the filenames.
        /// </summary>

        class Context : IDisposable
        {
            String m_sQueue;
            public UInt64 m_nNodes;
            public FileStream m_fsQueue;

            String m_sNext;
            public UInt64 m_nNextNodes;
            public FileStream m_fsNext;
            public BinaryFormatter m_bf;

            public void initialize(String q1, String q2)
            {
                m_sQueue = q1;
                m_nNodes = 0;
                if (m_fsQueue != null)
                    m_fsQueue.Close();

                m_sNext = q2;
                m_nNextNodes = 0;
                if (m_fsNext != null)
                    m_fsNext.Close();
                m_fsNext = new FileStream(m_sNext, FileMode.Create);

                m_bf = new BinaryFormatter();
            }

            /// <summary>
            /// After processing all of the nodes in one level, and writing the
            /// resulting children nodes to the next file, we are now ready to
            /// process the next file. Therefore, we simply swap the input for
            /// the output file stream.
            /// </summary>

            public void nextLevel()
            {
                String sTemp = m_sQueue;
                m_sQueue = m_sNext;
                m_sNext = sTemp;

                m_nNodes = m_nNextNodes;
                m_nNextNodes = 0;

                if (m_fsQueue != null)
                    m_fsQueue.Close();
                if (m_fsNext != null)
                    m_fsNext.Close();

                m_fsQueue = new FileStream(m_sQueue, FileMode.Open, FileAccess.Read);
                m_fsNext = new FileStream(m_sNext, FileMode.Create, FileAccess.Write);
            }
            public void write(WorldState tws)
            {
                m_bf.Serialize(m_fsNext, tws);
                ++m_nNextNodes;
            }
            public void clear()
            {
                m_fsQueue.Close();
                m_fsNext.Close();
            }

            public void Dispose()
            {
                this.m_fsQueue.Dispose();
                this.m_fsNext.Dispose();
            }
        };

        /// <summary>
        /// The size of an enumerative pattern database based on our member
        /// variables is simply as many permutations as possible given the
        /// number of agents and the free locations. Note there may be some
        /// additional constant overhead associated with this class.
        /// </summary>
        /// <returns>An estimate of the amount of memory required for this
        /// pattern database in units of bytes.</returns>
        public override UInt64 estimateSize()
        {
            return (m_vPermutations[0] * m_Problem.m_nLocations +
                (ulong) (sizeof(UInt64) * m_vPermutations.Length));
        }

        /// <summary>
        /// The initialization function accepts a problem instance describing
        /// the original full problem, and a list of agents for which this
        /// pattern database will be responsible for. This must be called prior
        /// to a call to build the pattern database.
        /// </summary>
        /// <param name="pi">A description of the problem instance.</param>
        /// <param name="vAgents">The agents that we should be responsible for.
        /// Each entry in the list is an index to ProblemInstance.m_vAgents
        /// pointing to which agents we care about.</param>
        public override void init(ProblemInstance pi, List<uint> vAgents)
        {
            base.init(pi, vAgents);
            computePermutations();
        }

        /// <summary>
        /// Builds the pattern database, storing the heuristic table in memory.
        /// </summary>
        public override void build()
        {
            /**
             * Create the subproblem pertaining to this additive pattern
             * database. We do this by taking the problem instance and swapping
             * the initial state with the goal. We will also save a copy of our
             * m_vAgents data structure, because during the building process 
             * our state already is a projection.
             */

            WorldState goal =
                new WorldState(m_Problem.m_vAgents, m_vAgents);
            foreach (AgentState ags in goal.allAgentsState)
                ags.swapCurrentWithGoal();
            List<uint> vBackup = m_vAgents;
            m_vAgents = new List<uint>(goal.allAgentsState.Length);
            for (uint i = 0; i < goal.allAgentsState.Length; ++i)
                m_vAgents.Add(i);

            /**
             * Initialize variables and insert the root node into our queue. We
             * use Byte.MaxValue to symbolize that an entry in our heuristic
             * table is uninitialized. The first time that we initialize an
             * entry in the table is also the first time that we encounter the
             * particular state, which is also the shortest path to that state
             * because we are conducting an uninformed breadth-first search.
             */

            m_vTable = new Byte[m_vPermutations[0] * (m_Problem.m_nLocations + 1)];
            for (int i = 0; i < m_vTable.Length; ++i)
                m_vTable[i] = Byte.MaxValue;
            Context c = new Context();
            c.initialize("q1.tmp", "q2.tmp");
            m_vTable[hash(goal)] = 0;
            c.write(goal);
            c.nextLevel();
            while (c.m_nNodes > 0)
            {
                for (ulong n = 0; n < c.m_nNodes; ++n)
                {
                    /**
                     * Get the next node, generate its children and write the
                     * children to the next queue file. I had previously
                     * thought that since we are doing an uninformed breadth-
                     * first search, the first time we generate a node we would
                     * also have the shortest path to that node. Unfortunately,
                     * for this particular domain, this is not true.
                     */

                    List<WorldState> vChildren = new List<WorldState>();
                    WorldState tws = (WorldState)c.m_bf.Deserialize(c.m_fsQueue);
                    UInt32 nHashParent = hash(tws);
                    if (nHashParent == 8184)
                    {
                        bool b;
                        b = true;
                    }

                    this.Expand(tws, vChildren);

                    foreach (WorldState i in vChildren)
                    {
                        UInt32 nHash = hash(i);

                        /**
                         * We store only the difference in heuristic value
                         * between the single agent shortest path heuristic and
                         * our pattern database heuristic. The hope is that the
                         * resulting value will always fit within the minimum
                         * and maximum ranges of a single byte.
                         */

                        Byte nCandidateValue;
                        if (m_bOffsetFromSingleShortestPath)
                        {
                            int nSingleAgentShortestPath = 0;
                            foreach (var a in i.allAgentsState)
                                nSingleAgentShortestPath +=
                                    this.m_Problem.GetSingleAgentShortestPath(a.agent.agentNum, a.pos_X, a.pos_Y);
                            int nDifference = i.g - nSingleAgentShortestPath;
                            Debug.Assert(nDifference >= 0);
                            Debug.Assert(nDifference < Byte.MaxValue);
                            nCandidateValue = (Byte)nDifference;
                        }
                        else
                        {
                            Debug.Assert(i.g < Byte.MaxValue);
                            nCandidateValue = (Byte)i.g;
                        }
                        if (nCandidateValue < m_vTable[nHash])
                        {
                            c.write(i);
                            m_vTable[nHash] = nCandidateValue;
                        }
                    }
                }
                c.nextLevel();
            }
            c.clear();
            m_vAgents = vBackup;
        }

        /// <summary>
        /// Returns the heuristic estimate for the subset of agents of the
        /// given state that this pattern database is responsible for.
        /// </summary>
        /// <param name="s">The current state.</param>
        /// <returns>The PDB entry for the given state.</returns>
        public override uint h(WorldState s)
        {
            var nSingleAgentShortestPath = 0;
            if (m_bOffsetFromSingleShortestPath)
                foreach (var a in m_vAgents)
                {
                    nSingleAgentShortestPath +=
                        this.m_Problem.GetSingleAgentShortestPath(s.allAgentsState[a].agent.agentNum,
                        s.allAgentsState[a].pos_X, s.allAgentsState[a].pos_Y);
                }
            return (m_vTable[hash(s)] + (uint) nSingleAgentShortestPath);
        }

        /// <summary>
        /// While working on the original search problem, we have to only pay
        /// attention to a subset of all of the agents in the current state.
        /// Specifically, we only consider the agents listed in m_vAgents,
        /// which are the agents that we've built the current pattern database
        /// for. This differs from the buildHash function, which hashes all
        /// agents in the given state.
        /// </summary>
        /// <param name="s">A state which includes all of the agents in the
        /// original search problem.</param>
        /// <returns>An index into the table of heuristic values.</returns>
        uint hash(WorldState s)
        {

            /**
             * This function works as follows. We have enumerated all of the
             * empty locations in our grid from 0 up to
             * (ProblemInstance.m_nLocations - 1). The first agent is allowed
             * to be placed in any location. The second agent is allowed to be
             * placed in any locations except for where the first one is
             * placed, etc. This means that after placing the first agent, we
             * must relabel all of the remaining empty locations from 0 up to
             * (ProblemInstance.m_nLocations - 2), etc.
             */

            UInt32 nHash = 0;
            for (int i = 0; i < m_vAgents.Count; ++i)
            {

                /**
                 * Compute the cardinality of the agent. That is, compute j
                 * where the agent is in the jth empty location. This requires
                 * us to keep figure out how many other agents have been placed
                 * in positions previous to our current position.
                 */

                Int32 nCard1 = m_Problem.getCardinality(s.allAgentsState[m_vAgents[i]]);
                Int32 nPreceding = 0;
                for (int j = 0; j < i; ++j)
                {
                    Int32 nCard2 = m_Problem.getCardinality(s.allAgentsState[m_vAgents[j]]);
                    if (nCard2 < nCard1)
                        ++nPreceding;
                }
                UInt32 nCardinality = (UInt32)(nCard1 - nPreceding);
                nHash += nCardinality * (UInt32)m_vPermutations[i];
            }
            return (nHash);
        }

        /// <summary>
        /// When building the pattern database, it is assumed that the root
        /// node of our search tree contains state information only about the
        /// agents that we care about. Therefore, our hash function should make
        /// use of all agents contained in the given state. This differs from
        /// the hash function that is used during the original search problem,
        /// which is required to extract out the subset of agents for which our
        /// pattern database is built for. This is the difference between this
        /// hash function and the one previously defined.
        /// </summary>
        /// <param name="s">A state which include only agents that pertain to
        /// this pattern database.</param>
        /// <returns>An index into the table of heuristic values.</returns>
#if false
        uint buildHash(Travor_WorldState s)
        {

            /**
             * This function works as follows. We have enumerated all of the
             * empty locations in our grid from 0 up to
             * (ProblemInstance.m_nLocations - 1). The first agent is allowed
             * to be placed in any location. The second agent is allowed to be
             * placed in any locations except for where the first one is
             * placed, etc. This means that after placing the first agent, we
             * must relabel all of the remaining empty locations from 0 up to
             * (ProblemInstance.m_nLocations - 2), etc.
             */

            UInt32 nHash = 0;
            for (int i = 0; i < s.allAgentsState.Length; ++i)
            {

                /**
                 * Compute the cardinality of the agent. That is, compute j
                 * where the agent is in the jth empty location. This requires
                 * us to keep figure out how many other agents have been placed
                 * in positions previous to our current position.
                 */

                Int32 nCard1 = m_Problem.getCardinality(s.allAgentsState[i]);
                Int32 nPreceding = 0;
                for (int j = 0; j < i; ++j)
                {
                    Int32 nCard2 = m_Problem.getCardinality(s.allAgentsState[j]);
                    if (nCard2 < nCard1)
                        ++nPreceding;
                }
                UInt32 nCardinality = (UInt32)(nCard1 - nPreceding);
                nHash += nCardinality * (UInt32)m_vPermutations[i];
            }
            return (nHash);
        }
#endif
        /// <summary>
        /// We precompute the total number of permutations for a given number
        /// of agents. We refer to these precomputed values when we call our
        /// hash function.
        /// </summary>
        void computePermutations()
        {
            m_vPermutations = new UInt64[m_vAgents.Count];
            m_vPermutations[m_vPermutations.Length - 1] = 1;
            for(var i = m_vPermutations.Length - 2; i >= 0; --i)
                m_vPermutations[i] = m_vPermutations[i + 1] * (UInt64) (m_Problem.m_nLocations - (i + 1));
        }
    }
}
