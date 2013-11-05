using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections;
using System.IO;
using System.Diagnostics;

namespace CPF_experement
{   
    /// <summary>
    /// This is a implementation of the classic A* algorithm for the MAPF problem.
    /// </summary>
    public class ClassicAStar : IDnCSolver 
    {
        protected ProblemInstance instance;
        protected HeuristicCalculator heuristic;
        public BinaryHeap openList;
        public HashTable_C closedList;
        public int solutionDepth;
        public int expanded;
        public int generated;
        public int totalCost;
        public int numOfAgents;
        protected int maxCost;
        protected int expandedFullStates;
        protected HashSet<TimedMove> illegalMoves;
        protected HashSet_U<DnCConstraint> constraintList;
        protected Run runner;
        protected bool foundGoal;
        public WorldState goal; // This will contain the goal node if such was found
        protected int minDepth;
        protected int internalConflictCount;
        protected int externalConflictCount;
        protected List<DnCConstraint>[] mustConstraints;

        ///// <summary>
        ///// This variable is used for the recursive Expand() procedure, to record the moves of previous agents.
        ///// </summary>
        //protected HashSet<Move> currentMoves;

        /// <summary>
        /// Default constructor.
        /// </summary>
        public ClassicAStar()
        {
            this.closedList = new HashTable_C();
            this.openList = new BinaryHeap();
        }

        /// <summary>
        /// Setup the relevant datastructures for a run.
        /// </summary>
        public virtual void Setup(ProblemInstance problemInstance)
        {
            this.instance = problemInstance;
            this.heuristic = this.CreateHeuristic();
            WorldState root = this.CreateSearchRoot();
            root.h = (int)this.heuristic.h(root);
            this.closedList.Add(root);
            this.openList.Add(root);
            this.expanded = 0;
            this.generated = 0;
            expandedFullStates = 0;
            this.totalCost = 0;
            this.solutionDepth = 0;
            this.goal = null;
            this.numOfAgents = problemInstance.m_vAgents.Length;

            // Store parameters used by Trevor's Independant Detection algorithm
            if (problemInstance.parameters.ContainsKey(Trevor.MAXIMUM_COST_KEY))
                this.maxCost = (int)(problemInstance.parameters[Trevor.MAXIMUM_COST_KEY]);
            else
                this.maxCost = int.MaxValue;
            if (problemInstance.parameters.ContainsKey(Trevor.ILLEGAL_MOVES_KEY))
                this.illegalMoves = (HashSet<TimedMove>)(problemInstance.parameters[Trevor.ILLEGAL_MOVES_KEY]);
            else
                this.illegalMoves = null;
            this.minDepth = 0;
        }

        /// <summary>
        /// Creates the initial state from which the search will start. 
        /// This will be the first state to be inserted to OPEN.
        /// </summary>
        /// <returns>The root of the search tree</returns>
        protected virtual WorldState CreateSearchRoot()
        {
            return new WorldState(this.instance.m_vAgents);
        }

        /// <summary>
        /// Clears the relevant datastructures and variables to free memory usage.
        /// </summary>
        public void Clear()
        {
            this.openList.Clear();
            this.closedList.Clear();
            this.mustConstraints = null;
            this.illegalMoves = null;
        }


        /// <summary>
        /// This method creates the object that will calculate the heuristics.
        /// </summary>
        /// <returns></returns>
        protected HeuristicCalculator CreateHeuristic()
        {
            SingleShortestPath sic = new SingleShortestPath();

            List<uint> agentList = new List<uint>();
            for(int i=0;i<this.instance.m_vAgents.Length;i++)
                agentList.Add((uint)i);
            sic.init(this.instance, agentList);

            sic.build();
            return sic;
        }

        public virtual String GetName() { return "A*"; }

        public WorldState GetGoal() { return this.goal; }

        public int GetSolutionCost() { return this.totalCost; }

        /// <summary>
        /// Prints statistics of a single run to the given output. 
        /// </summary>
        public virtual void OutputStatistics(TextWriter output)
        {
            output.Write(this.expanded + Run.RESULTS_DELIMITER);
            output.Write(this.generated + Run.RESULTS_DELIMITER);
            output.Write("N/A" + Run.RESULTS_DELIMITER);
            output.Write("N/A" + Run.RESULTS_DELIMITER);
            output.Write("N/A" + Run.RESULTS_DELIMITER);
            //output.Write(solutionDepth + Run.RESULTS_DELIMITER);
            // output.Write(expandedFullStates + Run.RESULTS_DELIMITER);
            // output.Write("NA"/*Process.GetCurrentProcess().VirtualMemorySize64*/ + Run.RESULTS_DELIMITER);
        }

        /// <summary>
        /// Runs the algorithm until the problem is solved or memory/time is exhausted
        /// </summary>
        /// <returns>True if solved</returns>
        public bool Solve(Run runner)
        {
            this.runner = runner;
            this.foundGoal = false;
            WorldState currentNode;
            this.solutionDepth = ((WorldState)openList.Peek()).h;

            while (openList.Count > 0)
            {
                // Check if max time has been exceeded
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    totalCost = Constants.TIMEOUT_COST;
                    this.generated = this.closedList.Count;
                    Console.WriteLine("Out of time");
                    this.Clear();
                    return false;
                }
                currentNode = (WorldState)openList.Remove();
                // Check if node is the goal
                if (currentNode.GoalTest(minDepth))
                {
                    this.totalCost = currentNode.g;
                    this.goal = currentNode;
                    this.solutionDepth = this.totalCost - this.solutionDepth;
                    this.Clear();
                    return true;
                }

                //WorldState ff = (WorldState)openList.Peek();
                //if (ff !=null && ff.allAgentsState[0].pos_X == 0 && ff.allAgentsState[0].pos_Y == 1 &&
                //    ff.allAgentsState[1].pos_X == 0 && ff.allAgentsState[1].pos_Y == 0 &&
                //    ff.allAgentsState[2].pos_X == 0 && ff.allAgentsState[2].pos_Y == 2 &&
                //    ff.allAgentsState[3].pos_X == 0 && ff.allAgentsState[3].pos_Y == 1)
                //    Console.ReadLine();

                // Expand
                expanded++;
                Expand(currentNode);
            }
            totalCost = Constants.NO_SOLUTION_COST;
            this.generated = this.closedList.Count;
            this.Clear();
            return false;
        }

        /// <summary>
        /// Expand a given node. This includes:
        /// - Generating all possible children
        /// - Inserting them to OPEN
        /// - Insert the generated nodes to the hashtable of nodes, currently implmented together with the closed list.
        /// </summary>
        /// <param name="node"></param>
        public virtual bool Expand(WorldState node)
        {
            //Debug.Print("Expanding node " + node);
            Expand(node, 0,runner, new HashSet<Move>());
            return true;
        }
        
        /// <summary>
        /// Expands a node. This is done recursively - generating agent possibilities one at a time.
        /// This includes:
        /// - Generating the childern
        /// - Inserting them into OPEN
        /// - Insert node into CLOSED
        /// TODO: Make expand not recursive to gain speedup of runtime.
        /// </summary>
        /// <param name="currentNode"></param>
        protected virtual void Expand(WorldState currentNode, int agentIndex,Run runner,HashSet<Move> currentMoves)
        {
            if (runner.ElapsedMilliseconds() > Constants.MAX_TIME || this.foundGoal)
                return;
            WorldState prev = currentNode.prevStep;
            if (agentIndex == 0) // If this is the first agent that moves
            {
                prev = currentNode; 
            }
            if (agentIndex == instance.m_vAgents.Length) // If all the agents have moved
            {

                currentNode.h = (int)this.heuristic.h(currentNode);
                currentNode.makespan++;
                currentNode.CalculateG();
                if (currentNode.h + currentNode.g <= this.maxCost)
                {

                    if (instance.parameters.ContainsKey(Trevor.CONFLICT_AVOIDENCE))
                    {
                        currentNode.potentialConflictsCount = currentNode.prevStep.potentialConflictsCount;
                        currentNode.potentialConflictsCount += currentNode.conflictsCount(((HashSet<TimedMove>)instance.parameters[Trevor.CONFLICT_AVOIDENCE]));
                    }

                    if (instance.parameters.ContainsKey(CBS_LocalConflicts.INTERNAL_CAT))
                    {
                        currentNode.dncInternalConflictsCount = currentNode.prevStep.dncInternalConflictsCount;
                        currentNode.dncInternalConflictsCount += currentNode.conflictsCount(((HashSet_U<TimedMove>)instance.parameters[CBS_LocalConflicts.INTERNAL_CAT]));
                    }

                    //if in closed list
                    if (this.closedList.Contains(currentNode) == true)
                    {
                        WorldState inClosedList = (WorldState)this.closedList[currentNode];
                        //if g is smaller than remove the old world state
                        if (inClosedList.g > currentNode.g || (inClosedList.g == currentNode.g && (inClosedList.potentialConflictsCount > currentNode.potentialConflictsCount || ( inClosedList.potentialConflictsCount == currentNode.potentialConflictsCount && inClosedList.dncInternalConflictsCount > currentNode.dncInternalConflictsCount ))))
                        {
                            closedList.Remove(inClosedList); //than remove state
                            openList.Remove(inClosedList);
                        }
                    }
                    if (this.closedList.Contains(currentNode) == false)
                    {
                        this.closedList.Add(currentNode);
                        this.generated++;

                        //if (currentNode.h + currentNode.g + currentNode.potentialConflictsCount == currentNode.prevStep.h + currentNode.prevStep.g + currentNode.prevStep.potentialConflictsCount )
                        //{
                        //    if (currentNode.h == 0)
                        //    {
                        //        this.foundGoal = true;
                        //        this.openList.Add(currentNode);
                        //    }
                        //    else
                        //    {
                        //        expanded++;
                        //        Expand(currentNode);
                        //    }
                        //}
                        //else
                            this.openList.Add(currentNode);
                    }
                }
                return;
            }

            // Try all legal moves of the agents
            TimedMove agentLocation = new TimedMove();
            DnCConstraint nextStepLocation = new DnCConstraint();
            int deltaX;
            int deltaY;
            int posX = currentNode.allAgentsState[agentIndex].pos_X;
            int posY = currentNode.allAgentsState[agentIndex].pos_Y;
            WorldState childNode;
            bool ileagel;
            for (int op = 0; op < WorldState.operators.GetLength(0); op++)
            {
                deltaX = WorldState.operators[op, 0];
                deltaY = WorldState.operators[op, 1];

                agentLocation.setup(posX + deltaX, posY + deltaY, WorldState.operators[op, 2], currentNode.makespan + 1);
                if (IsValidMove(agentLocation, currentMoves) == false)
                    continue;
                if (this.constraintList != null)
                {
                    nextStepLocation.init(instance.m_vAgents[agentIndex].agent.agentNum, posX + deltaX, posY + deltaY, currentNode.makespan + 1, WorldState.operators[op, 2]);
                    if (constraintList.Contains(nextStepLocation))
                        continue;
                    if (mustConstraints != null && mustConstraints.Length > currentNode.makespan + 1 && mustConstraints[currentNode.makespan + 1] != null)
                    {
                        ileagel = false;
                        foreach (DnCConstraint con in mustConstraints[currentNode.makespan + 1])
                        {
                            if (con.vioalates(instance.m_vAgents[agentIndex].agent.agentNum, posX + deltaX, posY + deltaY, currentNode.makespan + 1, WorldState.operators[op, 2]))
                            {
                                ileagel = true;
                                break;
                            }
                        }
                        if (ileagel)
                            continue;
                    }
                }

                currentMoves.Add(agentLocation);
                childNode = new WorldState(currentNode);
                childNode.allAgentsState[agentIndex].move(op);
                childNode.prevStep = prev;
                Expand(childNode, agentIndex + 1, runner, currentMoves);
                currentMoves.Remove(agentLocation);
            }
            
        }

        /// <summary>
        /// Check if the move is valid, i.e. not colliding into walls or other agents.
        /// </summary>
        /// <param name="possibleMove">The move to check if possible</param>
        /// <param name="agentNum">The agent for which to check the move</param>
        /// <returns>true, if the move is possible.</returns>
        protected bool IsValidMove(TimedMove possibleMove, HashSet<Move> currentMoves)
        {
            int moveDirection = possibleMove.direction;
            if (this.illegalMoves != null) 
            {
                
                possibleMove.direction = -1;
                if (this.illegalMoves.Contains(possibleMove))
                {
                    possibleMove.direction = moveDirection;
                    return false;
                }
                possibleMove.direction = moveDirection;
                possibleMove.setOppositeMove();
                if (this.illegalMoves.Contains(possibleMove))
                {
                    possibleMove.setOppositeMove();
                    return false;
                }
                possibleMove.setOppositeMove();
            }

            // If the tile is not free (out of the grid or with an obstacles)
            if (instance.IsValidForMove(possibleMove)==false)
                return false;

            // If previous move of another agent will collide with this move
            

            // Check if the past move arrived at the same location (direction is not important here)
            possibleMove.direction = Move.NO_DIRECTION;
            if (currentMoves.Contains(possibleMove))
            {
                possibleMove.direction = moveDirection;
                return false;
            }
            possibleMove.direction = moveDirection;
            // Check if the past move arrived in the opposite direction (direction here is improtant)
            possibleMove.setOppositeMove();
            if (currentMoves.Contains(possibleMove))
            {
                possibleMove.setOppositeMove();
                return false;
            }
            possibleMove.setOppositeMove();

            return true;
        }

        /// <summary>
        /// Returns the found plan, or null if no plan was found.
        /// </summary>
        /// <returns></returns>
        public virtual Plan GetPlan()
        {
            return new Plan(this.GetGoal());
        }

        public virtual SinglePlan[] getSinglePlans()
        {
            return SinglePlan.getSinglePlans(this.GetGoal());
        }
        
        /// <summary>
        /// Prints the actual solutions. Mainly for debug purposes
        /// </summary>
        /// <param name="end"></param>
        private void PrintSolution(WorldState end)
        {
            int step = 0;
            Console.WriteLine("solution back to front");
            while (end != null)
            {
                Console.WriteLine("step " + step);
                Console.WriteLine(end.ToString());
                step++;
                end = end.prevStep;
            }
        }

        public int getExpanded() { return this.expanded; }
        public int getGenerated() { return this.generated; }
        public int getSolutionDepth() { return this.solutionDepth; }
        public virtual int getTrueNagativeCount() { return -1; }
        public virtual int getNodesPassedPruningCounter() { return expandedFullStates; }
        public long getMemuryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }

        //DnC SOLVER
        public void Setup(ProblemInstance problemInstance, int minDepth)
        {
            this.Setup(problemInstance);
            this.minDepth = minDepth;
            this.internalConflictCount = 0;
            this.externalConflictCount = 0;
            if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.CONSTRAINTS))
                this.constraintList = (HashSet_U<DnCConstraint>)problemInstance.parameters[CBS_LocalConflicts.CONSTRAINTS];
            if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.CONSTRAINTSP))
            {
                List<DnCConstraint> lc = (List<DnCConstraint>)problemInstance.parameters[CBS_LocalConflicts.CONSTRAINTSP];
                if (lc != null && lc.Count > 0)
                {
                    mustConstraints = new List<DnCConstraint>[lc[lc.Count - 1].getTimeStep() + 1];
                    foreach (DnCConstraint con in lc)
                    {
                        if (mustConstraints[con.getTimeStep()] == null)
                            mustConstraints[con.getTimeStep()] = new List<DnCConstraint>();
                        mustConstraints[con.getTimeStep()].Add(con);
                    }
                }
            }
        }
        public int getHighLeveExpanded() { return 0; }
        public int getHighLeveGenerated() { return 0; }
        public int getLowLevelExpanded() { return expanded; }
        public int getLowLevelGenerated() { return generated; }
        public int getMaxGroupSize() { return numOfAgents; }
    }
}

