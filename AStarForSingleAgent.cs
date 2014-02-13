using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CPF_experiment
{
    class AStarForSingleAgent
    {
        ProblemInstance instance;
        int agentNum;
        BinaryHeap openList;
        HashTable_C closedList;
        int solutionCost;
        HashSet<DnCConstraint> constraintsInGroup;
        Plan plan;
        int minDepth;
        HashSet<TimedMove> conflictTableThisGroup;
        HashSet<TimedMove> conflictTableOtherGroups;
        public int expanded;
        public ushort externalConflicts;

        public AStarForSingleAgent()
        {
            this.openList = new BinaryHeap();
            this.closedList = new HashTable_C();
        }

        public virtual void Setup(ProblemInstance problemInstance, int agentNum, HashSet<DnCConstraint> constraints, HashSet<TimedMove> avoid, int minDepth = -1)
        {
            this.agentNum = agentNum;
            this.instance = problemInstance;
            this.constraintsInGroup = constraints;
            AgentState root = problemInstance.m_vAgents[agentNum];
            root.h = problemInstance.GetSingleAgentShortestPath(root);
            root.potentialConflicts = 0;
            this.closedList.Clear();
            this.openList.Clear();
            this.closedList.Add(root);
            this.openList.Add(root);
            this.minDepth = minDepth;
            this.conflictTableThisGroup = avoid;
            this.expanded = 0;
            externalConflicts = 0;

            // Store parameters used by Trevor's Independant Detection algorithm
            if (problemInstance.parameters.ContainsKey(Trevor.CONFLICT_AVOIDENCE))
                this.conflictTableOtherGroups = (HashSet<TimedMove>)(problemInstance.parameters[Trevor.CONFLICT_AVOIDENCE]);
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
                if (currentNode.atGoal() && currentNode.currentStep > this.minDepth)
                {
                    this.solutionCost = currentNode.currentStep;
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
            int next_X;
            int next_Y;
            TimedMove nextMove=new TimedMove();
            DnCConstraint nextStepLocation = new DnCConstraint();

            for (int direction = 0; direction < 5; direction++)
            {
                next_X = currentNode.pos_X + WorldState.operators[direction, 0];
                next_Y = currentNode.pos_Y + WorldState.operators[direction, 1];
                nextMove.setup(next_X, next_Y, direction, currentNode.currentStep + 1);
                if (instance.IsValidForMove(nextMove))
                {
                    nextStepLocation.init(agentNum, next_X, next_Y, currentNode.currentStep + 1,direction);
                    if (constraintsInGroup.Contains(nextStepLocation) == false)
                    {
                        nextStep = new AgentState(currentNode);
                        nextStep.prev = currentNode;
                        nextStep.move(direction);
                        nextStep.h = Math.Max(instance.GetSingleAgentShortestPath(nextStep),(minDepth - nextStep.currentStep));
                        nextStep.potentialConflicts = currentNode.potentialConflicts;
                        nextStep.potentialConflictsID = currentNode.potentialConflictsID;

                        nextMove.direction = -1;
                        if (conflictTableThisGroup.Contains(nextMove))
                            nextStep.potentialConflicts++;
                        if (conflictTableOtherGroups!=null && conflictTableOtherGroups.Contains(nextMove))
                            nextStep.potentialConflictsID++;

                        nextMove.direction = direction;
                        nextMove.setOppositeMove();

                        if (conflictTableThisGroup.Contains(nextMove))
                            nextStep.potentialConflicts++;
                        if (conflictTableOtherGroups != null && conflictTableOtherGroups.Contains(nextMove))
                            nextStep.potentialConflictsID++;

                        if (this.closedList.Contains(nextStep) == true)
                        {
                            AgentState inClosedList = (AgentState)this.closedList[nextStep];
                            //if g is smaller then remove the old world state
                            if (inClosedList.potentialConflictsID > nextStep.potentialConflictsID || (inClosedList.potentialConflictsID == nextStep.potentialConflictsID &&
                                inClosedList.potentialConflicts > nextStep.potentialConflicts))
                            {
                                closedList.Remove(inClosedList); //than remove state
                                openList.Remove(inClosedList);
                            }
                        }
                        if (this.closedList.Contains(nextStep) == false)
                        {
                            this.openList.Add(nextStep);
                            this.closedList.Add(nextStep);
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
        HashTable_C closedList;
        public int conflictsCount;
        public int clearCount;
        int optimalSol;

         public SetPotentialConflictsForAgent()
        {
            this.openList = new BinaryHeap();
            this.closedList = new HashTable_C();
        }

        public virtual void Setup(ProblemInstance problemInstance, int agentNum)
        {
            this.agentNum = agentNum;
            this.instance = problemInstance;
            AgentState root = problemInstance.m_vAgents[agentNum];
            root.h = problemInstance.GetSingleAgentShortestPath(root);
            this.closedList.Clear();
            this.openList.Clear();
            this.closedList.Add(root);
            this.openList.Add(root);
            optimalSol=root.h;
            clearCount=-1;
            conflictsCount=0;
        }

        public void Solve(HashSet<CoordinateForConflictRatio> reservd, int orderOfConflict)
        {
            AgentState currentNode;
            CoordinateForConflictRatio c1, c2;

            while (openList.Count > 0)
            {
                currentNode = (AgentState)openList.Remove();

                c1=new CoordinateForConflictRatio(instance,currentNode);
                c2 = new CoordinateForConflictRatio(c1);
                c2.direction = 0;
                if (reservd.Contains(c2))
                    conflictsCount++;
                else
                {
                    reservd.Add(c2);
                    if (c1.direction != 0)
                    {
                        c1.setOppositeMove();
                        c1.cardinality = instance.m_vCardinality[c1.x, c1.y];
                        if (reservd.Contains(c1))
                            conflictsCount++;
                        else
                        {
                            clearCount++;
                            c1.setOppositeMove();
                            c1.cardinality = instance.m_vCardinality[c1.x, c1.y];
                            reservd.Add(c1);
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
            int next_X;
            int next_Y;
            TimedMove nextMove = new TimedMove();

            for (int direction = 0; direction < 5; direction++)
            {
                next_X = currentNode.pos_X + WorldState.operators[direction, 0];
                next_Y = currentNode.pos_Y + WorldState.operators[direction, 1];
                nextMove.setup(next_X, next_Y, direction, currentNode.currentStep + 1);
                if (instance.IsValidForMove(nextMove))
                {
                    nextStep = new AgentState(currentNode);
                    nextStep.prev = currentNode;
                    nextStep.move(direction);
                    nextStep.h = instance.GetSingleAgentShortestPath(nextStep);

                    if (nextStep.currentStep + nextStep.h <= optimalSol + orderOfConflict)
                    {
                        if (this.closedList.Contains(nextStep) == true)
                        {
                            continue;
                        }
                        this.openList.Add(nextStep);
                        this.closedList.Add(nextStep);
                    }
                }
            }
        }
    }
}
