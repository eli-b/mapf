using System.Collections.Generic;
using System.Linq;

namespace CPF_experiment
{
    /// <summary>
    /// This class tries to find a solution to a problem instance of a given cost. 
    /// (this is used in the CostTreeSearchSolver)
    /// </summary>
    abstract class CostTreeNodeSolver
    {
        protected MDD[] allMDDs;
        int maxCost;
        AgentState[] startingPos;
        public int totalCost;
        public int generated;
        public int expanded;
        protected Run runner;
        protected ProblemInstance problem;

        public int caViolations;

        public static int matchCounter; // For debugging

        public CostTreeNodeSolver(ProblemInstance problem, Run runner)
        {
            this.runner = runner;
            this.problem = problem;
            this.allMDDs = new MDD[problem.GetNumOfAgents()];
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="problem"></param>
        /// <param name="runner"></param>
        /// <param name="agentNums"></param>
        /// <param name="costsNode">Of all the agents, not just the ones selected</param>
        public CostTreeNodeSolver(ProblemInstance problem, Run runner, int[] agentNums, CostTreeNode costsNode)
        {
            this.runner = runner;
            this.problem = problem;
            this.allMDDs = new MDD[agentNums.Length];

            this.Setup(agentNums, costsNode);
        }

        /// <summary>
        /// Automatically calls Setup with the given costsNode
        /// </summary>
        /// <param name="problem"></param>
        /// <param name="costNode">TODO: Maybe just pass the array of costs here?</param>
        /// <param name="runner"></param>
        public CostTreeNodeSolver(ProblemInstance problem, CostTreeNode costNode, Run runner) // Make sure agent numbers are in the correct order
            : this(problem, runner)
        {
            this.Setup(costNode);
        }

        public virtual void Setup(CostTreeNode costsNode)
        {
            this.startingPos = problem.m_vAgents;
            this.totalCost = costsNode.costs.Sum();
            this.maxCost = costsNode.costs.Max();

            for (int i = 0; i < this.allMDDs.Length; i++)
            {
                this.allMDDs[i] = new MDD(i, startingPos[i].agent.agentNum, startingPos[i].lastMove,
                                     costsNode.costs[i], maxCost, startingPos.Length, problem);
            }
            this.generated = 0;
            // TODO: Not clearing the expanded count?
            CostTreeNodeSolver.matchCounter = 0;
        }

        public virtual void Setup(int[] agentNums, CostTreeNode costsNode)
        {
            this.startingPos = new AgentState[agentNums.Length];
            int index = 0;
            foreach (var agentNum in agentNums)
            {
                while (problem.m_vAgents[index].agent.agentNum != agentNum)
                    ++index;
                this.startingPos[index] = this.problem.m_vAgents[index];
            }
            this.totalCost = costsNode.costs.Sum();
            this.maxCost = costsNode.costs.Max();

            for (int i = 0; i < this.allMDDs.Length; i++)
            {
                this.allMDDs[i] = new MDD(agentNums[i], startingPos[i].agent.agentNum, startingPos[i].lastMove,
                                     costsNode.costs[i], maxCost, startingPos.Length, problem);
            }
            this.generated = 0;
            // TODO: Not clearing the expanded count?
            CostTreeNodeSolver.matchCounter = 0;
        }

        /// <summary>
        /// Tries to find a solution for the agents with the given cost.
        /// </summary>
        /// <returns>The solution if found or null otherwise</returns>
        public abstract SinglePlan[] Solve(Dictionary<TimedMove, List<int>> conflictTable, Dictionary<TimedMove, List<int>> CBS_CAT);

        public int getGenerated() { return generated; }
    }


    public class CostTreeNode
    {
        public int[] costs;

        public CostTreeNode(int[] costs)
        {
            this.costs = costs;
        }

        public void Expand(Queue<CostTreeNode> openList, HashSet<CostTreeNode> closedList)
        {
            for (int j = 0; j < costs.Length; j++)
            {
                int[] newCosts = this.costs.ToArray<int>();
                newCosts[j]++;
                var child = new CostTreeNode(newCosts);
                if (!closedList.Contains(child))
                {
                    closedList.Add(child);
                    openList.Enqueue(child);
                }
            }
        }

        public int Sum(int from, int to)
        {
            int ans = 0;
            for (int i = from; i < to; i++)
            {
                ans += costs[i];
            }
            return ans;
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int ans = 0;
                for (int i = 0; i < costs.Length; i++)
                {
                    ans += costs[i] * Constants.PRIMES_FOR_HASHING[i % Constants.PRIMES_FOR_HASHING.Length];
                }
                return ans;
            }
        }

        public override bool Equals(object obj)
        {
            CostTreeNode check = (CostTreeNode)obj;
            return this.costs.SequenceEqual<int>(check.costs);
        }
    }

    /// <summary>
    /// Use this one!
    /// </summary>
    class CostTreeNodeSolverOldMatching : CostTreeNodeSolver
    {
        int syncSize;
        public CostTreeNodeSolverOldMatching(ProblemInstance problem, Run runner) : base(problem, runner) {}
        public CostTreeNodeSolverOldMatching(ProblemInstance problem, CostTreeNode costNode, Run runner, int syncSize) : base(problem, costNode, runner) { this.syncSize = syncSize; }
        
        public void Setup(CostTreeNode costNode, int syncSize)
        {
            base.Setup(costNode);
            this.syncSize = syncSize;
        }

        /// <summary>
        /// Prunes the individual agent MDDs and, if possible, matches them to return a non-conflicting configuration of
        /// paths of the given costs.
        /// </summary>
        /// <param name="conflictTable"></param>
        /// <param name="CBS_CAT"></param>
        /// <returns>
        /// null if no solution is found.
        /// </returns>
        public override SinglePlan[] Solve(Dictionary<TimedMove, List<int>> conflictTable, Dictionary<TimedMove, List<int>> CBS_CAT)
        {
            if (this.Prune()) // Everything was pruned
                return null;

            CostTreeSearchSolver.passed++;
            AStarMDD findSolution = new AStarMDD(allMDDs, runner, conflictTable, CBS_CAT);
            
            SinglePlan[] ans = findSolution.Solve();
            this.generated = findSolution.generated;
            this.expanded = findSolution.expanded;
            this.caViolations = findSolution.conflictAvoidanceViolations;
            return ans;
        }

        /// <summary>
        /// Prunes the individual agent MDDs and prepares data for matching them
        /// </summary>
        /// <returns>Whether everything was pruned</returns>
        public bool Prune()
        {
            for (int i = allMDDs.Length - 1; i >= 0; i--)
            {
                for (int j = i + 1; j < allMDDs.Length; j++)
                {
                    MDD.PruningDone pruningDone = MDD.PruningDone.NOTHING;
                    pruningDone = allMDDs[i].SyncMDDs(allMDDs[j], this.syncSize == 3);

                    if (pruningDone == MDD.PruningDone.EVERYTHING)
                        return true;
                }
            }
            if (allMDDs[0].levels == null)
                return true;
            return false;
        }
    }

    class CostTreeNodeSolverDDBF : CostTreeNodeSolver
    {
        public CostTreeNodeSolverDDBF(ProblemInstance problem, Run runner) : base(problem, runner) { }
        public CostTreeNodeSolverDDBF(ProblemInstance problem, CostTreeNode costNode, Run runner) : base(problem, costNode, runner) { }
        public override void Setup(CostTreeNode costNode)
        {
            base.Setup(costNode);
        }
        public override SinglePlan[] Solve(Dictionary<TimedMove, List<int>> conflictTable, Dictionary<TimedMove, List<int>> CBS_CAT)
        {
            for (int i = 0; i < allMDDs.Length; i++)
                if (allMDDs[i].levels == null)
                    return null;
            AStarMDD findSolution = new AStarMDD(allMDDs, runner, conflictTable, CBS_CAT);
            SinglePlan[] ans = findSolution.Solve();
            generated = findSolution.generated;
            expanded = findSolution.expanded;
            caViolations = findSolution.conflictAvoidanceViolations;
            return ans;
        }
    }

    class CostTreeNodeSolverKSimpleMatching : CostTreeNodeSolver
    {
        int maxGroupChecked;
        public CostTreeNodeSolverKSimpleMatching(ProblemInstance problem, Run runner) : base(problem, runner) { }
        public CostTreeNodeSolverKSimpleMatching(ProblemInstance problem, CostTreeNode costNode, Run runner, int maxGroupChecked) : base(problem, costNode, runner) { this.maxGroupChecked = maxGroupChecked; }
        public void setup(CostTreeNode costNode,int maxGroupChecked)
        {
            base.Setup(costNode);
            this.maxGroupChecked = maxGroupChecked;
        }
        public override SinglePlan[] Solve(Dictionary<TimedMove, List<int>> conflictTable, Dictionary<TimedMove, List<int>> CBS_CAT)
        {
            AStarMDD findSolution;
            SinglePlan[] subCheck;
            MDD[] match;
            MddMatchAndPrune matcher = new MddMatchAndPrune(runner);

            foreach (MDD checkValid in allMDDs)
            {
                if (checkValid.levels == null)
                    return null;
            }

            if (maxGroupChecked >= 2)
            {
                match = new MDD[2];
                for (int i = allMDDs.Length - 1; i >= 0; i--)
                {
                    for (int j = i + 1; j < allMDDs.Length; j++)
                    {
                        match[0] = allMDDs[i];
                        match[1] = allMDDs[j];
                        //matcher.initialize(match);

                        //if (matcher.pruneMDDs() == false)

                        findSolution = new AStarMDD(match, runner, conflictTable,CBS_CAT);

                        subCheck = findSolution.Solve();
                        if (subCheck == null || subCheck[0] == null)
                        {
                            return null;
                        }
                    }
                }
            }
            if (maxGroupChecked >= 3)
            {
                match = new MDD[3];
                for (int i = allMDDs.Length - 2; i >= 0; i--)
                {
                    for (int j = i + 1; j < allMDDs.Length - 1; j++)
                    {
                        for (int t = j + 1; t < allMDDs.Length; t++)
                        {
                            match[0] = allMDDs[i];
                            match[1] = allMDDs[j];
                            match[2] = allMDDs[t];
                            //matcher.initialize(match);

                            //if (matcher.pruneMDDs() == false)
                            findSolution = new AStarMDD(match, runner,conflictTable,CBS_CAT);

                            subCheck = findSolution.Solve();
                            if (subCheck == null || subCheck[0] == null)
                            {
                                return null;
                            }
                        }
                    }
                }
            }
            if (maxGroupChecked >= 4)
            {
                match = new MDD[4];
                for (int i = allMDDs.Length - 3; i >= 0; i--)
                {
                    for (int j = i + 1; j < allMDDs.Length - 2; j++)
                    {
                        for (int t = j + 1; t < allMDDs.Length - 1; t++)
                        {
                            for (int m = t + 1; m < allMDDs.Length; m++)
                            {  
                                match[0] = allMDDs[i];
                                match[1] = allMDDs[j];
                                match[2] = allMDDs[t];
                                match[3] = allMDDs[m];
                                //matcher.initialize(match);

                                //if (matcher.pruneMDDs() == false)
                                findSolution = new AStarMDD(match, runner,conflictTable,CBS_CAT);

                                subCheck = findSolution.Solve();
                                if (subCheck == null || subCheck[0] == null)
                                {
                                    return null;
                                }

                            }

                        }
                    }
                }
            }
            CostTreeSearchSolver.passed++;
            if (allMDDs[0].levels == null)
                return null;
            findSolution = new AStarMDD(allMDDs, runner, conflictTable,CBS_CAT);
            SinglePlan[] ans = findSolution.Solve();
            generated = findSolution.generated;
            expanded = findSolution.expanded;
            caViolations = findSolution.conflictAvoidanceViolations;
            return ans;
        }
    }

    class CostTreeNodeSolverRepeatedMatching : CostTreeNodeSolver
    {
        int syncSize;
        public CostTreeNodeSolverRepeatedMatching(ProblemInstance problem, Run runner) : base(problem, runner) { }
        public CostTreeNodeSolverRepeatedMatching(ProblemInstance problem, CostTreeNode costNode, Run runner, int syncSize) : base(problem, costNode, runner) { this.syncSize = syncSize; }
        public void setup(CostTreeNode costNode, int syncSize)
        {
            base.Setup(costNode);
            this.syncSize = syncSize;
        }
        public override SinglePlan[] Solve(Dictionary<TimedMove, List<int>> conflictTable, Dictionary<TimedMove, List<int>> CBS_CAT)
        {
            MDD[] match = new MDD[2];
            bool Converging = true;
            int[] changed = new int[allMDDs.Length];
            int currentIteration = 0;
            MDD.PruningDone conflictStatus = MDD.PruningDone.NOTHING;

            while (Converging)
            {
                currentIteration++;
                Converging = false;

                for (int i = allMDDs.Length - 1; i >= 0; i--)
                {
                    for (int j = i + 1; j < allMDDs.Length; j++)
                    {
                        if (changed[i] >= currentIteration - 1 || changed[j] >= currentIteration - 1) // If at least one of the two MDDs was changed during the last iteration
                        {
                            conflictStatus = allMDDs[i].SyncMDDs(allMDDs[j], this.syncSize == 3);

                            if (conflictStatus == MDD.PruningDone.EVERYTHING)
                            {
                                return null;
                            }

                            else if (conflictStatus == MDD.PruningDone.SOME)
                            {
                                changed[i] = currentIteration;
                                Converging = true;
                            }

                            conflictStatus = allMDDs[i].SyncMDDs(allMDDs[j], this.syncSize == 3);

                            if (conflictStatus == MDD.PruningDone.EVERYTHING)
                            {
                                return null;
                            }

                            else if (conflictStatus == MDD.PruningDone.SOME)
                            {
                                changed[i] = currentIteration;
                                Converging = true;
                            }
                        }
                    }
                }
            }
            CostTreeSearchSolver.passed++;
            if (allMDDs[0].levels == null)
                return null;
            AStarMDD findSolution = new AStarMDD(allMDDs, runner, conflictTable, CBS_CAT);
            SinglePlan[] ans = findSolution.Solve();
            generated = findSolution.generated;
            expanded = findSolution.expanded;
            caViolations = findSolution.conflictAvoidanceViolations;
            return ans;
        }
    }
}
