using System;
using System.Collections.Generic;  // For List
using System.IO;  // For BinaryWriter, FileStream
using System.Diagnostics;  // For Trace.Assert

namespace mapf;

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
    byte[] table;

    /// <summary>
    /// permutations[i] represents the number of permutations of the
    /// remaining agents after placing the ith agent. For example, if we
    /// had 10 agents, and we've already placed 3 of them, then
    /// permutations[2] represents the number of permutations for the
    /// remaining 7 agents. This precomputed table depends on the number
    /// of free locations in the board and is used as a perfect hash 
    /// function that maps a state in the search space to an integer.
    /// </summary>
    ulong[] permutations;

    /// <summary>
    /// Determines whether or not we will internally represent heuristic
    /// values as offsets from the single shortest path heuristic.
    /// </summary>
    bool offsetFromSingleShortestPath = true;

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
        string queue;
        public ulong nodes;
        public FileStream fsQueue;

        string next;
        public ulong nextNodesCount;
        public FileStream fsNext;
        public BinaryWriter binaryWriter;

        public void Initialize(string q1, string q2)
        {
            queue = q1;
            nodes = 0;
            if (fsQueue != null)
                fsQueue.Close();

            next = q2;
            nextNodesCount = 0;
            if (fsNext != null)
                fsNext.Close();
            fsNext = new FileStream(next, FileMode.Create);
            binaryWriter = new BinaryWriter(fsNext);
        }

        /// <summary>
        /// After processing all of the nodes in one level, and writing the
        /// resulting children nodes to the next file, we are now ready to
        /// process the next file. Therefore, we simply swap the input for
        /// the output file stream.
        /// </summary>
        public void NextLevel()
        {
            string sTemp = queue;
            queue = next;
            next = sTemp;

            nodes = nextNodesCount;
            nextNodesCount = 0;

            if (fsQueue != null)
                fsQueue.Close();
            if (fsNext != null)
            {
                binaryWriter.Close();
                fsNext.Close();
            }

            fsQueue = new FileStream(queue, FileMode.Open, FileAccess.Read);
            fsNext = new FileStream(next, FileMode.Create, FileAccess.Write);
            binaryWriter = new BinaryWriter(fsNext);
        }
        public void Write(WorldState tws)
        {
            //binaryWriter.Write(tws);  // TODO: Adapt from BinaryFormatter to BinaryWriter
            nextNodesCount++;
        }
        public void Clear()
        {
            fsQueue.Close();
            binaryWriter.Close();
            fsNext.Close();
        }

        public void Dispose()
        {
            this.fsQueue.Dispose();
            this.binaryWriter.Dispose();
            this.fsNext.Dispose();
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
    public override ulong estimateSize()
    {
        return permutations[0] * problem.numLocations + (ulong) (sizeof(ulong) * permutations.Length);
    }

    /// <summary>
    /// The initialization function accepts a problem instance describing
    /// the original full problem, and a list of agents for which this
    /// pattern database will be responsible for. This must be called prior
    /// to a call to build the pattern database.
    /// </summary>
    /// <param name="pi">A description of the problem instance.</param>
    /// <param name="agentsToConsider">The agents that we should be responsible for.
    /// Each entry in the list is an index to ProblemInstance.agents
    /// pointing to which agents we care about.</param>
    public override void Init(ProblemInstance pi, List<uint> agentsToConsider)
    {
        base.Init(pi, agentsToConsider);
        computePermutations();
    }

    /// <summary>
    /// Builds the pattern database, storing the heuristic table in memory.
    /// </summary>
    public override void build()
    {
        // Create the subproblem pertaining to this additive pattern
        // database. We do this by taking the problem instance and swapping
        // the initial state with the goal. We will also save a copy of our
        // agents data structure, because during the building process 
        // our state already is a projection.

        WorldState goal = new WorldState(problem.agents, agentsToConsider);
        foreach (AgentState ags in goal.allAgentsState)
            ags.SwapCurrentWithGoal();
        List<uint> vBackup = agentsToConsider;
        agentsToConsider = new List<uint>(goal.allAgentsState.Length);
        for (uint i = 0; i < goal.allAgentsState.Length; ++i)
            agentsToConsider.Add(i);

        // Initialize variables and insert the root node into our queue. We
        // use Byte.MaxValue to symbolize that an entry in our heuristic
        // table is uninitialized. The first time that we initialize an
        // entry in the table is also the first time that we encounter the
        // particular state, which is also the shortest path to that state
        // because we are conducting an uninformed breadth-first search.

        table = new Byte[permutations[0] * (problem.numLocations + 1)];
        for (int i = 0; i < table.Length; ++i)
            table[i] = Byte.MaxValue;
        Context c = new Context();
        c.Initialize("q1.tmp", "q2.tmp");
        table[hash(goal)] = 0;
        c.Write(goal);
        c.NextLevel();
        while (c.nodes > 0)
        {
            for (ulong n = 0; n < c.nodes; ++n)
            {
                // Get the next node, generate its children and write the
                // children to the next queue file. I had previously
                // thought that since we are doing an uninformed breadth-
                // first search, the first time we generate a node we would
                // also have the shortest path to that node. Unfortunately,
                // for this particular domain, this is not true.

                List<WorldState> vChildren = new List<WorldState>();
                /*
                WorldState tws = (WorldState)c.binaryWriter.Deserialize(c.fsQueue);  // TODO: Adapt from BinaryFormatter to BinaryWriter
                UInt32 nHashParent = hash(tws);

                this.Expand(tws, vChildren);
                */

                foreach (WorldState i in vChildren)
                {
                    UInt32 nHash = hash(i);

                    // We store only the difference in heuristic value
                    // between the single agent shortest path heuristic and
                    // our pattern database heuristic. The hope is that the
                    // resulting value will always fit within the minimum
                    // and maximum ranges of a single byte.

                    Byte nCandidateValue;
                    if (offsetFromSingleShortestPath)
                    {
                        int nSingleAgentShortestPath = 0;
                        foreach (var a in i.allAgentsState)
                            nSingleAgentShortestPath += this.problem.GetSingleAgentOptimalCost(a);
                        int nDifference = i.g - nSingleAgentShortestPath;
                        Trace.Assert(nDifference >= 0);
                        Trace.Assert(nDifference < Byte.MaxValue);
                        nCandidateValue = (Byte)nDifference;
                    }
                    else
                    {
                        Trace.Assert(i.g < Byte.MaxValue);
                        nCandidateValue = (Byte)i.g;
                    }
                    if (nCandidateValue < table[nHash])
                    {
                        c.Write(i);
                        table[nHash] = nCandidateValue;
                    }
                }
            }
            c.NextLevel();
        }
        c.Clear();
        agentsToConsider = vBackup;
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
        if (offsetFromSingleShortestPath)
            foreach (var a in agentsToConsider)
            {
                nSingleAgentShortestPath +=
                    this.problem.GetSingleAgentOptimalCost(s.allAgentsState[a]);
            }
        return (table[hash(s)] + (uint) nSingleAgentShortestPath);
    }

    /// <summary>
    /// While working on the original search problem, we have to only pay
    /// attention to a subset of all of the agents in the current state.
    /// Specifically, we only consider the agents listed in agents,
    /// which are the agents that we've built the current pattern database
    /// for. This differs from the buildHash function, which hashes all
    /// agents in the given state.
    /// </summary>
    /// <param name="s">A state which includes all of the agents in the
    /// original search problem.</param>
    /// <returns>An index into the table of heuristic values.</returns>
    uint hash(WorldState s)
    {
        // This function works as follows. We have enumerated all of the
        // empty locations in our grid from 0 up to
        // (ProblemInstance.locations - 1). The first agent is allowed
        // to be placed in any location. The second agent is allowed to be
        // placed in any locations except for where the first one is
        // placed, etc. This means that after placing the first agent, we
        // must relabel all of the remaining empty locations from 0 up to
        // (ProblemInstance.locations - 2), etc.

        uint hash = 0;
        for (int i = 0; i < agentsToConsider.Count; ++i)
        {
            // Compute the cardinality of the agent. That is, compute j
            // where the agent is in the jth empty location. This requires
            // us to keep figure out how many other agents have been placed
            // in positions previous to our current position.

            int card1 = problem.GetCardinality(s.allAgentsState[agentsToConsider[i]].lastMove);
            int preceding = 0;
            for (int j = 0; j < i; ++j)
            {
                int nCard2 = problem.GetCardinality(s.allAgentsState[agentsToConsider[j]].lastMove);
                if (nCard2 < card1)
                    ++preceding;
            }
            uint nCardinality = (uint)(card1 - preceding);
            hash += nCardinality * (uint)permutations[i];
        }
        return hash;
    }

#if false
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
    uint buildHash(WorldState s)
    {

        /**
            * This function works as follows. We have enumerated all of the
            * empty locations in our grid from 0 up to
            * (ProblemInstance.locations - 1). The first agent is allowed
            * to be placed in any location. The second agent is allowed to be
            * placed in any locations except for where the first one is
            * placed, etc. This means that after placing the first agent, we
            * must relabel all of the remaining empty locations from 0 up to
            * (ProblemInstance.locations - 2), etc.
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

            Int32 nCard1 = problem.getCardinality(s.allAgentsState[i]);
            Int32 nPreceding = 0;
            for (int j = 0; j < i; ++j)
            {
                Int32 nCard2 = problem.getCardinality(s.allAgentsState[j]);
                if (nCard2 < nCard1)
                    ++nPreceding;
            }
            UInt32 nCardinality = (UInt32)(nCard1 - nPreceding);
            nHash += nCardinality * (UInt32)permutations[i];
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
        permutations = new UInt64[agentsToConsider.Count];
        permutations[permutations.Length - 1] = 1;
        for(var i = permutations.Length - 2; i >= 0; --i)
            permutations[i] = permutations[i + 1] * (UInt64) (problem.numLocations - (i + 1));
    }
}
