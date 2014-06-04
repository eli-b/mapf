using System;
using System.Collections.Generic;
using System.Collections;
using System.IO;
using System.Diagnostics;

namespace CPF_experiment
{
    /// <summary>
    /// Runs Silver's CA*.
    /// </summary>
    [Obsolete("Not maintained for about 2-3 months.")]
    class Silver 
    {
        Run runner;
        HashSet<Move> RT;
        AgentState[] allAgentsState;
        Hashtable parked;
        int[] allPathCost;
        int maxPathCost;
        public int expanded;
        public int generated;
        public int totalTime;
        private ProblemInstance problem;

        public Silver()
        {
            RT = new HashSet<Move>();
            parked = new Hashtable();
        }

        public void Clear()
        {
            this.RT.Clear();
            this.parked.Clear();
        }

        public void Setup(ProblemInstance instance)
        {
            this.problem = instance;
            this.allAgentsState = instance.m_vAgents;
            this.allPathCost = new int[this.allAgentsState.Length];
            this.maxPathCost = 0;
            this.expanded = 0;
            this.generated = 0;
            this.totalTime = 0;
        }

        public String GetName() { return "Silver"; }

        public Plan GetPlan() { throw new NotSupportedException("TODO"); }

        public int GetSolutionCost() { return this.totalTime; }

        public void OutputStatisticsHeader(TextWriter output)
        {
            output.Write("expanded" + Run.RESULTS_DELIMITER);
            output.Write("generated" + Run.RESULTS_DELIMITER);
        }
        
        /// <summary>
        /// Prints statistics of a single run to the given output. 
        /// </summary>
        public void OutputStatistics(TextWriter output)
        {
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

        public bool Solve(Run aRunner)
        {
            this.runner = aRunner;
            foreach (AgentState agent in allAgentsState)
            {
                if (!singleAgentA_Star(agent))
                {
                    totalTime = -1;
                    Console.WriteLine("Expanded - " + expanded);
                    Console.WriteLine("Generated - " + generated);
                    Console.WriteLine("No solution found");
                    return false;
                }
            }
            printSum();
            return true;
        }

        private bool singleAgentA_Star(AgentState agent)
        {
            BinaryHeap openList = new BinaryHeap(); // TODO: Safe to use OpenList here instead?
            HashSet<AgentState> closedList = new HashSet<AgentState>();
            openList.Add(agent);
            AgentState temp = agent;
            bool valid;
            while (openList.Count > 0)
            {
                if (this.runner.ElapsedMilliseconds() > 120000)
                {
                    return false;
                }
                temp = (AgentState)openList.Remove();
                if (temp.h == 0)
                {
                    valid = true;
                    for (int i = temp.lastMove.time ; i <= maxPathCost; i++)
                    {
                        if (RT.Contains(new TimedMove(temp.lastMove.x, temp.lastMove.y, Move.Direction.NO_DIRECTION, i)))
                            valid = false;
                    }
                    if (valid)
                    {
                        reservePath(temp);
                        totalTime += temp.lastMove.time;
                        //printPath(temp);
                        parked.Add(new Move(temp.lastMove.x, temp.lastMove.y, Move.Direction.NO_DIRECTION), temp.lastMove.time);
                        return true;
                    }
                }
                expanded++;
                expendNode(temp, openList, closedList);
            }
            return false;
        }
        private void reservePath(AgentState end)
        {
            AgentState temp = end;
            while (temp != null)
            {
                RT.Add(new TimedMove(temp.lastMove.x, temp.lastMove.y, temp.lastMove.direction, temp.lastMove.time));
                allPathCost[temp.agent.agentNum]++;
                temp = temp.prev;
            }
            if (allPathCost[end.agent.agentNum] > maxPathCost)
                maxPathCost = allPathCost[end.agent.agentNum];
        }
        private void expendNode(AgentState node, BinaryHeap openList, HashSet<AgentState> closedList)
        {
            foreach (TimedMove move in node.lastMove.GetNextMoves(Constants.ALLOW_DIAGONAL_MOVE))
            {
                if (isValidMove(move))
                {
                    AgentState temp = new AgentState(node);
                    AgentState tempCL = temp;
                    temp.prev = node;
                    if (move.time > maxPathCost)
                    {
                        tempCL = new AgentState(temp);
                        tempCL.lastMove.time = maxPathCost;
                    }
                    if (!closedList.Contains(tempCL))
                    {
                        closedList.Add(tempCL);
                        temp.lastMove.time = move.time;
                        openList.Add(temp);
                        generated++;
                    }
                }
            }

        }

        private bool isValidMove(TimedMove move)
        {
            if (this.problem.IsValid(move))
            {
                if (move.isColliding(RT) == false)
                {
                    Move key = new Move(move.x, move.y, Move.Direction.NO_DIRECTION);
                    if (!parked.Contains(key) || (int)(parked[key]) > move.time)
                        return true;
                }
            }
            //switch (step.direction)
            //{
            //    case 0:
            //        return true;
            //    case 1:
            //        return safeFromCollision(step.pos_X + 1, step.pos_Y, step.step, 3);
            //    case 2:
            //        return safeFromCollision(step.pos_X, step.pos_Y-1, step.step, 4);
            //    case 3:
            //        return safeFromCollision(step.pos_X - 1, step.pos_Y, step.step, 1);
            //    case 4:
            //        return safeFromCollision(step.pos_X , step.pos_Y+1, step.step, 2);
            //    case 5:
            //        return safeFromCollision(step.pos_X + 1, step.pos_Y-1, step.step, 7);
            //    case 6:
            //        return safeFromCollision(step.pos_X - 1, step.pos_Y-1, step.step, 8);
            //    case 7:
            //        return safeFromCollision(step.pos_X - 1, step.pos_Y+1, step.step, 5);
            //    case 8:
            //        return safeFromCollision(step.pos_X + 1, step.pos_Y+1, step.step, 6);
            //}
            return false;
        }
        
        //private bool safeFromCollision(int pos_x, int pos_Y, int step, short direction)
        //{
        //    Reservation res = (Reservation)RT[new Reservation(pos_x, pos_Y, step, direction)];
        //    if (res != null && res.direction == direction)
        //        return false;
        //    return true;
        //}
        
        private void printPath(AgentState final)
        {
            Console.WriteLine(final.agent.ToString());
            Console.WriteLine("-----------------------");
            while (final != null)
            {
                Console.WriteLine(final.ToString());
                final = final.prev;
            }
            Console.WriteLine();
        }
        
        private void printSum()
        {
            Console.WriteLine("Expanded - " + expanded);
            Console.WriteLine("Generated - " + generated);
            Console.WriteLine("Total cost - " + totalTime);
        }
        
        public int getExpanded() { return this.expanded; }
        public int getGenerated() { return this.generated; }
        public int getSolutionDepth() { return -1; } // Lazy...
        public long getMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
    }
}
