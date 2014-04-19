using System;
using System.Collections.Generic;

namespace CPF_experiment
{
    class AStarForSingleAgent
    {
        ProblemInstance instance;
        int agentNum;
        BinaryHeap openList;
        Dictionary<AgentState, AgentState> closedList;
        int solutionCost;
        HashSet<CbsConstraint> constraintsInGroup;
        Plan plan;
        int minDepth;
        HashSet<TimedMove> conflictTableThisGroup;
        HashSet<TimedMove> conflictTableOtherGroups;
        public int expanded;
        public ushort externalConflicts;

        public AStarForSingleAgent()
        {
            this.openList = new BinaryHeap();
            this.closedList = new Dictionary<AgentState, AgentState>();
        }

        public virtual void Setup(ProblemInstance problemInstance, int agentNum, HashSet<CbsConstraint> constraints, HashSet<TimedMove> avoid, int minDepth = -1)
        {
            this.agentNum = agentNum;
            this.instance = problemInstance;
            this.constraintsInGroup = constraints;
            AgentState root = problemInstance.m_vAgents[agentNum];
            root.h = problemInstance.GetSingleAgentShortestPath(root);
            root.potentialConflicts = 0;
            this.closedList.Clear();
            this.openList.Clear();
            this.closedList.Add(root, root);
            this.openList.Add(root);
            this.minDepth = minDepth;
            this.conflictTableThisGroup = avoid;
            this.expanded = 0;
            externalConflicts = 0;

            // Store parameters used by Trevor's Independant Detection algorithm
            if (problemInstance.parameters.ContainsKey(Trevor.CONFLICT_AVOIDANCE))
                this.conflictTableOtherGroups = (HashSet<TimedMove>)(problemInstance.parameters[Trevor.CONFLICT_AVOIDANCE]);
            else
                this.conflictTableOtherGroups = null;
        }

        public int GetSolutionCost() { return this.solutionCost; }

        public Plan getPlan()
        {
            return plan;
        }

        public bool Solve(Run runner)
        {
            AgentState currentNode;
            while (openList.Count > 0)
            {
                // Check if max time has been exceeded
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    solutionCost = Constants.TIMEOUT_COST;
                    return false;
                }
                currentNode = (AgentState)openList.Remove();
                // Check if node is the goal
                if (currentNode.atGoal() && currentNode.last_move.time > this.minDepth)
                {
                    this.solutionCost = currentNode.last_move.time;
                    this.plan = new Plan(currentNode);
                    this.externalConflicts = currentNode.potentialConflictsID;
                    return true;
                }

                // Expand
                expanded++;
                expand(currentNode);
            }
            solutionCost = Constants.NO_SOLUTION_COST;
            return false;
        }

        private void expand(AgentState currentNode)
        {
            AgentState nextStep;
            CbsConstraint nextStepLocation = new CbsConstraint();

            foreach (TimedMove nextMove in currentNode.last_move.GetNextMoves(Constants.ALLOW_DIAGONAL_MOVE))
            {
                if (instance.IsValid(nextMove))
                {
                    if (constraintsInGroup.Contains(nextStepLocation) == false)
                    {
                        nextStep = new AgentState(currentNode);
                        nextStep.prev = currentNode;
                        nextStep.move(nextMove);
                        nextStep.h = Math.Max(instance.GetSingleAgentShortestPath(nextStep),
                                              minDepth - nextStep.last_move.time);
                        nextStep.potentialConflicts = currentNode.potentialConflicts;
                        nextStep.potentialConflictsID = currentNode.potentialConflictsID;

                        if (nextMove.isColliding(conflictTableThisGroup))
                            nextStep.potentialConflicts++;
                        if (nextMove.isColliding(conflictTableOtherGroups))
                            nextStep.potentialConflictsID++;
                        
                        if (this.closedList.ContainsKey(nextStep) == true)
                        {
                            AgentState inClosedList = this.closedList[nextStep];
                            //if g is smaller then remove the old world state
                            if (inClosedList.potentialConflictsID > nextStep.potentialConflictsID ||
                                (inClosedList.potentialConflictsID == nextStep.potentialConflictsID && inClosedList.potentialConflicts > nextStep.potentialConflicts))
                            {
                                closedList.Remove(inClosedList);
                                openList.Remove(inClosedList);
                            }
                        }
                        if (this.closedList.ContainsKey(nextStep) == false)
                        {
                            this.openList.Add(nextStep);
                            this.closedList.Add(nextStep, nextStep);
                        }
                    }
                }
            }
        }
    }

    class SetPotentialConflictsForAgent
    {
        ProblemInstance instance;
        int agentNum;
        BinaryHeap openList;
        Dictionary<AgentState, AgentState> closedList;
        public int conflictsCount;
        public int clearCount;
        int optimalSol;

         public SetPotentialConflictsForAgent()
        {
            this.openList = new BinaryHeap();
            this.closedList = new Dictionary<AgentState, AgentState>();
        }

        public virtual void Setup(ProblemInstance problemInstance, int agentNum)
        {
            this.agentNum = agentNum;
            this.instance = problemInstance;
            AgentState root = problemInstance.m_vAgents[agentNum];
            root.h = problemInstance.GetSingleAgentShortestPath(root);
            this.closedList.Clear();
            this.openList.Clear();
            this.closedList.Add(root, root);
            this.openList.Add(root);
            optimalSol = root.h;
            clearCount = -1;
            conflictsCount=0;
        }

        public void Solve(HashSet<CoordinateForConflictRatio> reserved, int orderOfConflict)
        {
            AgentState currentNode;
            CoordinateForConflictRatio c1, c2;

            while (openList.Count > 0)
            {
                currentNode = (AgentState)openList.Remove();

                c1 = new CoordinateForConflictRatio(instance, currentNode);
                c2 = new CoordinateForConflictRatio(c1);
                c2.direction = 0;
                if (reserved.Contains(c2))
                    conflictsCount++;
                else
                {
                    reserved.Add(c2);
                    if (c1.direction != 0)
                    {
                        c1.setOppositeMove();
                        c1.cardinality = instance.m_vCardinality[c1.x, c1.y];
                        if (reserved.Contains(c1))
                            conflictsCount++;
                        else
                        {
                            clearCount++;
                            c1.setOppositeMove();
                            c1.cardinality = instance.m_vCardinality[c1.x, c1.y];
                            reserved.Add(c1);
                        }
                    }
                    else
                        clearCount++;
                }

                expand(currentNode, orderOfConflict);
            }

        }

        private void expand(AgentState currentNode, int orderOfConflict)
        {
            AgentState nextStep;

            foreach (TimedMove nextMove in currentNode.last_move.GetNextMoves(Constants.ALLOW_DIAGONAL_MOVE))
            {
                if (instance.IsValid(nextMove))
                {
                    nextStep = new AgentState(currentNode);
                    nextStep.prev = currentNode;
                    nextStep.move(nextMove);
                    nextStep.h = instance.GetSingleAgentShortestPath(nextStep);

                    if (nextStep.last_move.time + nextStep.h <= optimalSol + orderOfConflict)
                    {
                        if (this.closedList.ContainsKey(nextStep) == true)
                        {
                            continue;
                        }
                        this.openList.Add(nextStep);
                        this.closedList.Add(nextStep, nextStep);
                    }
                }
            }
        }
    }
}
