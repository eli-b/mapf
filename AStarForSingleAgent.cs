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
            this.openList = new BinaryHeap(); // Can't use an OpenList because AStarForSingleAgent doesn't implement ISolver, and OpenList expects ISolver users.
            this.closedList = new Dictionary<AgentState, AgentState>();
        }

        public virtual void Setup(ProblemInstance problemInstance, int agentNum, HashSet<CbsConstraint> constraints, HashSet<TimedMove> avoid, int minDepth = -1)
        {
            this.agentNum = agentNum;
            this.instance = problemInstance;
            this.constraintsInGroup = constraints;
            AgentState root = problemInstance.m_vAgents[agentNum];
            root.h = problemInstance.GetSingleAgentOptimalCost(root);
            root.potentialConflicts = 0;
            this.closedList.Clear();
            this.openList.Clear();
            this.closedList.Add(root, root);
            this.openList.Add(root);
            this.minDepth = minDepth;
            this.conflictTableThisGroup = avoid;
            this.expanded = 0;
            externalConflicts = 0;

            // Store parameters used by Trevor's Independence Detection algorithm
            if (problemInstance.parameters.ContainsKey(Trevor.CONFLICT_AVOIDANCE))
                this.conflictTableOtherGroups = (HashSet<TimedMove>)(problemInstance.parameters[Trevor.CONFLICT_AVOIDANCE]);
            else
                this.conflictTableOtherGroups = null;
        }

        public int GetSolutionCost() { return this.solutionCost; }

        public Plan GetPlan()
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
                if (currentNode.AtGoal() && currentNode.lastMove.time > this.minDepth)
                {
                    this.solutionCost = currentNode.lastMove.time;
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

            foreach (TimedMove nextMove in currentNode.lastMove.GetNextMoves(Constants.ALLOW_DIAGONAL_MOVE))
            {
                if (instance.IsValid(nextMove))
                {
                    if (constraintsInGroup.Contains(nextStepLocation) == false)
                    {
                        nextStep = new AgentState(currentNode);
                        nextStep.prev = currentNode;
                        nextStep.MoveTo(nextMove);
                        nextStep.h = Math.Max(instance.GetSingleAgentOptimalCost(nextStep),
                                              minDepth - nextStep.lastMove.time);
                        nextStep.potentialConflicts = currentNode.potentialConflicts;
                        nextStep.potentialConflictsID = currentNode.potentialConflictsID;

                        if (nextMove.IsColliding(conflictTableThisGroup))
                            nextStep.potentialConflicts++;
                        if (nextMove.IsColliding(conflictTableOtherGroups))
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
}
