using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Diagnostics;

namespace mapf
{
    /// <summary>
    /// Runs David Silver's Cooperative A* (CA*).
    /// It doesn't find optimal solutions and isn't complete.
    /// </summary>
    class CooperativeAStar : IStatisticsCsvWriter, ISolver
    {
        /// <summary>
        /// The Reservation Table
        /// </summary>
        HashSet<TimedMove> reservationTable;
        AgentState[] allAgentsState;
        /// <summary>
        /// Maps locations (moves) to the time an agent parked there. From that point on they're
        /// blocked.
        /// </summary>
        Dictionary<Move, int> parked;
        int[] pathCosts;
        SinglePlan[] paths;
        int maxPathCostSoFar;
        public int expanded;
        public int generated;
        public int totalcost;
        private ProblemInstance problem;
        private Run runner;
        private int initialEstimate;

        public CooperativeAStar()
        {
            reservationTable = new HashSet<TimedMove>();
            parked = new Dictionary<Move, int>();
        }

        public string GetName()
        {
            return "CA*";
        }

        public void Clear()
        {
            this.reservationTable.Clear();
            this.parked.Clear();
            this.initialEstimate = 0;
            this.maxPathCostSoFar = 0;
            this.totalcost = 0;
            this.pathCosts = null;
            this.paths = null;
        }

        public void ClearStatistics()
        {
            this.expanded = 0;
            this.generated = 0;
        }

        public int GetExpanded()
        {
            return this.expanded;
        }

        public int GetGenerated()
        {
            return this.generated;
        }

        public long GetMemoryUsed()
        {
            return Process.GetCurrentProcess().VirtualMemorySize64;
        }

        public void Setup(ProblemInstance instance, Run runner)
        {
            this.Clear();
            this.ClearStatistics();
            this.problem = instance;
            this.allAgentsState = instance.agents;
            this.pathCosts = new int[this.allAgentsState.Length];
            this.paths = new SinglePlan[this.allAgentsState.Length];
            this.runner = runner;
        }

        public Plan GetPlan() { return new Plan(this.paths.TakeWhile(plan => plan != null)); }

        public int GetSolutionCost() { return this.totalcost; }

        public void OutputStatisticsHeader(TextWriter output)
        {
            output.Write(this.ToString() + " expanded" + Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " generated" + Run.RESULTS_DELIMITER);
        }

        public override string ToString()
        {
            return GetName();
        }

        public int GetSolutionDepth()
        {
            return this.totalcost - this.initialEstimate;
        }
        
        /// <summary>
        /// Prints statistics of a single run to the given output. 
        /// </summary>
        public void OutputStatistics(TextWriter output)
        {
            Console.WriteLine($"Expanded nodes: {this.expanded}");
            Console.WriteLine($"Generated nodes: {this.generated}");
            output.Write(this.expanded + Run.RESULTS_DELIMITER);
            output.Write(this.generated + Run.RESULTS_DELIMITER);
        }

        public int NumStatsColumns
        {
            get
            {
                return 2;
            }
        }

        public bool Solve()
        {
            foreach (AgentState agent in allAgentsState)
            {
                if (!singleAgentAStar(agent))
                {
                    this.totalcost = Constants.NO_SOLUTION_COST;
                    return false;
                }
            }
            return true;
        }

        public bool AddOneAgent(int index)
        {
            if (!singleAgentAStar(allAgentsState[index]))
            {
                this.totalcost = Constants.NO_SOLUTION_COST;
                return false;
            }
            return true;
        }

        private bool singleAgentAStar(AgentState agent)
        {
            AgentState.EquivalenceOverDifferentTimes = false;
            BinaryHeap<AgentState> openList = new BinaryHeap<AgentState>(); // TODO: Safe to use OpenList here instead?
            HashSet<AgentState> closedList = new HashSet<AgentState>();
            agent.h = this.problem.GetSingleAgentOptimalCost(agent);
            openList.Add(agent);
            AgentState node;
            this.initialEstimate += agent.h;
            TimedMove queryTimedMove = new TimedMove();

            while (openList.Count > 0)
            {
                if (this.runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    return false;
                }
                node = openList.Remove();
                if (node.h == 0)
                {
                    bool valid = true;
                    for (int i = node.lastMove.time ; i <= maxPathCostSoFar; i++)
                    {
                        queryTimedMove.setup(node.lastMove.x, node.lastMove.y, Move.Direction.NO_DIRECTION, i);
                        if (reservationTable.Contains(queryTimedMove))
                            valid = false;
                    }
                    if (valid)
                    {
                        this.paths[agent.agent.agentNum] = new SinglePlan(node);
                        reservePath(node);
                        totalcost += node.lastMove.time;
                        parked.Add(new Move(node.lastMove.x, node.lastMove.y, Move.Direction.NO_DIRECTION), node.lastMove.time);
                        return true;
                    }
                }
                expandNode(node, openList, closedList);
                expanded++;
            }
            return false;
        }

        private void reservePath(AgentState end)
        {
            AgentState node = end;
            while (node != null)
            {
                reservationTable.Add(new TimedMove(node.lastMove));
                pathCosts[node.agent.agentNum]++;
                node = node.prev;
            }
            if (pathCosts[end.agent.agentNum] > this.maxPathCostSoFar)
                this.maxPathCostSoFar = pathCosts[end.agent.agentNum];
        }

        private void expandNode(AgentState node, BinaryHeap<AgentState> openList, HashSet<AgentState> closedList)
        {
            foreach (TimedMove move in node.lastMove.GetNextMoves())
            {
                if (this.isValidMove(move))
                {
                    AgentState child = new AgentState(node);
                    child.prev = node;
                    child.MoveTo(move);
                    if (closedList.Contains(child) == false)
                    {
                        closedList.Add(child);
                        child.h = this.problem.GetSingleAgentOptimalCost(child);
                        openList.Add(child);
                        generated++;
                    }
                }
            }

        }

        private Move queryMove = new Move();

        private bool isValidMove(TimedMove move)
        {
            if (this.problem.IsValid(move) == false)
                return false;
            if (move.IsColliding(this.reservationTable))
                return false;
            this.queryMove.setup(move.x, move.y, Move.Direction.NO_DIRECTION);
            if (parked.ContainsKey(this.queryMove) && parked[this.queryMove] <= move.time)
                return false;
            return true;
        }
    }
}
