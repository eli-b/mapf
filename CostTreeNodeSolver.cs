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
        public int[] costs;
        public int generated;
        public int expanded;
        protected Run runner;
        protected ProblemInstance problem;

        public int caVaiulations;

        public static int matchCounter;//debugin

        public CostTreeNodeSolver(ProblemInstance problem, Run runner)
        {
            this.runner = runner;
            this.problem = problem;
            allMDDs = new MDD[problem.GetNumOfAgents()];
        }
        public CostTreeNodeSolver(ProblemInstance problem, CostTreeNode costNode, Run runner) //make sure agents numbers are in the corect order
        {
            this.runner = runner;
            AgentState agent;
            maxCost=0;
            allMDDs = new MDD[problem.GetNumOfAgents()];
            this.startingPos = problem.m_vAgents;
            this.costs = costNode.costs;
            this.problem = problem;

            maxCost = costNode.costs.Max();
            for (int i = 0; i < startingPos.Length; i++) 
			{
                agent=startingPos[i];
			    allMDDs[i]=new MDD(i ,agent.agent.agentNum,agent.pos_X,agent.pos_Y,costNode.costs[i],maxCost,startingPos.Length,problem);
			}

            matchCounter = 0;
        }

        public virtual void setup( CostTreeNode costNode)
        {
            AgentState agent;
            maxCost = 0;
            this.startingPos = problem.m_vAgents;
            this.costs = costNode.costs;

            maxCost = costNode.costs.Max();
            for (int i = 0; i < startingPos.Length; i++)
            {
                agent = startingPos[i];
                allMDDs[i] = new MDD(i, agent.agent.agentNum, agent.pos_X, agent.pos_Y, costNode.costs[i], maxCost, startingPos.Length, problem);
            }
            generated = 0;
            matchCounter = 0;
        }

        /// <summary>
        /// Tries to find a solution for the agents with the given cost.
        /// </summary>
        /// <returns>The solution if found or null otherwise</returns>
        public abstract LinkedList<Move>[] Solve(HashSet<TimedMove> conflictTable, HashSet_U<TimedMove> CBS_CAT);
        public int getGenerated() { return generated; }

    }


    class CostTreeNode
    {
        public int[] costs;

        public CostTreeNode(int[] costs)
        {
            this.costs = costs;
        }
        public void expandNode(LinkedList<CostTreeNode> openList, HashSet<CostTreeNode> closedList)
        {
            CostTreeNode temp;
            for (int j = 0; j < costs.Length; j++)
            {
                int[] newCosts = new int[costs.Length];
                for (int i = 0; i < costs.Length; i++)
                {
                    newCosts[i] = costs[i];
                }
                newCosts[j]++;
                temp = new CostTreeNode(newCosts);
                if (!closedList.Contains(temp))
                {
                    closedList.Add(temp);
                    openList.AddLast(temp);
                }
            }
        }
        public int sum(int from, int to)
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
                int[] primes = { 1, 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 71, 73, 79 };
                for (int i = 0; i < costs.Length; i++)
                {
                    ans += costs[i] * primes[i % primes.Length];
                }
                return ans;
            }
        }
        public override bool Equals(object obj)
        {
            CostTreeNode check = (CostTreeNode)obj;
            for (int i = 0; i < costs.Length; i++)
            {
                if (costs[i]!=check.costs[i])
                {
                    return false;
                }
            }
            return true;
        }
    }

    class CostTreeNodeSolverOldMatching : CostTreeNodeSolver
    {
        int syncSize;
        public CostTreeNodeSolverOldMatching(ProblemInstance problem, Run runner):base(problem,runner){}
        public CostTreeNodeSolverOldMatching(ProblemInstance problem, CostTreeNode costNode, Run runner, int syncSize) : base(problem, costNode, runner) { this.syncSize = syncSize; }
        public void setup(CostTreeNode costNode, int syncSize)
        {
            base.setup( costNode);
            this.syncSize = syncSize;
        }
        public override LinkedList<Move>[] Solve(HashSet<TimedMove> conflictTable, HashSet_U<TimedMove> CBS_CAT)
        {
            int notConflicting=1;
            for (int i = allMDDs.Length - 1; i >= 0; i--)
            {
                for (int j = i + 1; j < allMDDs.Length; j++)
                {
                    if (syncSize == 2)
                        notConflicting = allMDDs[i].sync2GDDs(allMDDs[j]);
                    else if(syncSize==3)
                        notConflicting = allMDDs[i].sync3GDDs(allMDDs[j], j);
                    //Run.resultsWriterdd.Write(matchCounter + ",");
                    //Run.resultsWriterdd.WriteLine();
                    if (notConflicting == 0)
                    {
                        return null;
                    }
                }
            }
            if (allMDDs[0].levels == null)
                return null;

            CostTreeSearchSolver.passed++;
            AStarMDD findSolution = new AStarMDD(allMDDs, runner, conflictTable, CBS_CAT);
            
            LinkedList<Move>[] ans =  findSolution.solve();
            generated = findSolution.generated;
            expanded = findSolution.expanded;
            caVaiulations = findSolution.conflictAvoidenceVaiulations;
            return ans;
        }
    }

    class CostTreeNodeSolverDDBF : CostTreeNodeSolver
    {
        public CostTreeNodeSolverDDBF(ProblemInstance problem, Run runner) : base(problem, runner) { }
        public CostTreeNodeSolverDDBF(ProblemInstance problem, CostTreeNode costNode, Run runner) : base(problem, costNode, runner) { }
        public override void setup(CostTreeNode costNode)
        {
            base.setup(costNode);
        }
        public override LinkedList<Move>[] Solve(HashSet<TimedMove> conflictTable, HashSet_U<TimedMove> CBS_CAT)
        {
            for (int i = 0; i < allMDDs.Length; i++)
                if (allMDDs[i].levels == null)
                    return null;
            AStarMDD findSolution = new AStarMDD(allMDDs, runner, conflictTable,CBS_CAT);
            LinkedList<Move>[] ans = findSolution.solve();
            generated = findSolution.generated;
            expanded = findSolution.expanded;
            caVaiulations = findSolution.conflictAvoidenceVaiulations;
            return ans;
        }
    }

    class CostTreeNodeSolverKSimpaleMatching : CostTreeNodeSolver
    {
        int maxGroupChecked;
        public CostTreeNodeSolverKSimpaleMatching(ProblemInstance problem, Run runner) : base(problem, runner) { }
        public CostTreeNodeSolverKSimpaleMatching(ProblemInstance problem, CostTreeNode costNode, Run runner, int maxGroupChecked) : base(problem, costNode, runner) { this.maxGroupChecked = maxGroupChecked; }
        public void setup(CostTreeNode costNode,int maxGroupChecked)
        {
            base.setup(costNode);
            this.maxGroupChecked = maxGroupChecked;
        }
        public override LinkedList<Move>[] Solve(HashSet<TimedMove> conflictTable, HashSet_U<TimedMove> CBS_CAT)
        {
            AStarMDD findSolution;
            LinkedList<Move>[] subCheck;
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

                        subCheck = findSolution.solve();
                        if (subCheck==null || subCheck[0] == null)
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

                            subCheck = findSolution.solve();
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

                                subCheck = findSolution.solve();
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
            LinkedList<Move>[] ans = findSolution.solve();
            generated = findSolution.generated;
            expanded = findSolution.expanded;
            caVaiulations = findSolution.conflictAvoidenceVaiulations;
            return ans;
        }
    }

    class CostTreeNodeSolverRepatedMatching : CostTreeNodeSolver
    {
        int syncSize;
        public CostTreeNodeSolverRepatedMatching(ProblemInstance problem, Run runner) : base(problem, runner) { }
        public CostTreeNodeSolverRepatedMatching(ProblemInstance problem, CostTreeNode costNode, Run runner, int syncSize) : base(problem, costNode, runner) { this.syncSize = syncSize; }
        public void setup(CostTreeNode costNode, int syncSize)
        {
            base.setup(costNode);
            this.syncSize = syncSize;
        }
        public override LinkedList<Move>[] Solve(HashSet<TimedMove> conflictTable, HashSet_U<TimedMove> CBS_CAT)
        {
            MDD[] match = new MDD[2];
            bool Converging = true;
            int[] changed = new int[allMDDs.Length];
            int currentIteration = 0;
            int conflictStatus=1;

            while (Converging)
            {
                currentIteration++;
                Converging = false;

                for (int i = allMDDs.Length - 1; i >= 0; i--)
                {
                    for (int j = i + 1; j < allMDDs.Length; j++)
                    {
                        if (changed[i] >= currentIteration - 1 || changed[j] >= currentIteration - 1)//if at least one of the two MDDs was changed during the last iteration
                        {
                            if (syncSize == 2)
                                conflictStatus = allMDDs[i].sync2GDDs(allMDDs[j]);
                            else if (syncSize == 3)
                                conflictStatus = allMDDs[i].sync3GDDs(allMDDs[j], j);

                            if (conflictStatus == 0)
                            {
                                return null;
                            }

                            else if (conflictStatus == 2)
                            {
                                changed[i] = currentIteration;
                                Converging = true;
                            }

                            if (syncSize == 2)
                                conflictStatus = allMDDs[i].sync2GDDs(allMDDs[j]);
                            else if (syncSize == 3)
                                conflictStatus = allMDDs[i].sync3GDDs(allMDDs[j], j);

                            if (conflictStatus == 0)
                            {
                                return null;
                            }

                            else if (conflictStatus == 2)
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
            AStarMDD findSolution = new AStarMDD(allMDDs, runner, conflictTable,CBS_CAT);
            LinkedList<Move>[] ans = findSolution.solve();
            generated = findSolution.generated;
            expanded = findSolution.expanded;
            caVaiulations = findSolution.conflictAvoidenceVaiulations;
            return ans;
        }
    }
    
}
